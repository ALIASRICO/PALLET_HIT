#!/usr/bin/env python3
"""
Standalone AG-95 Gripper Modbus Test Script
Tests each Modbus operation against the physical Dobot CR20.

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/dobot_ws/install/setup.bash

  # Run all tests:
  python3 test_gripper_modbus.py

  # Run specific test:
  python3 test_gripper_modbus.py --test init
  python3 test_gripper_modbus.py --test open
  python3 test_gripper_modbus.py --test close
  python3 test_gripper_modbus.py --test status
  python3 test_gripper_modbus.py --test full_cycle

Prerequisites:
  - dobot_bringup_v4 node running
  - Robot powered on and connected
  - Gripper physically connected to tool port
"""

import argparse
import logging
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from dobot_msgs_v4.srv import (
    EnableRobot,
    GetHoldRegs,
    ModbusClose,
    ModbusCreate,
    SetHoldRegs,
    SetToolPower,
)

# ---------------------------------------------------------------------------
# Constants — mirror depalletizer.py exactly
# ---------------------------------------------------------------------------
SRV_NS = '/dobot_bringup_ros2/srv'

MODBUS_IP       = '192.168.5.1'
MODBUS_PORT     = 60000
MODBUS_SLAVE_ID = 1
MODBUS_IS_RTU   = 1
MODBUS_INDEX    = 0

REG_INIT        = 256
REG_FORCE       = 257
REG_SPEED       = 258
REG_POSITION    = 259
REG_INIT_STATUS = 512
REG_STATUS      = 513
REG_ACTUAL_POS  = 514

INIT_VALUE      = 165
FORCE           = 50
SPEED           = 50
OPEN_POS        = 1000
CLOSE_POS       = 500

INIT_TIMEOUT    = 15.0
POLL_TIMEOUT    = 5.0
POLL_INTERVAL   = 0.1
INTER_CMD_DELAY = 0.01

# ---------------------------------------------------------------------------
# Status lookup
# ---------------------------------------------------------------------------
STATUS_MAP = {
    0: 'moving',
    1: 'obj_open (object detected while opening)',
    2: 'obj_close (object detected while closing = GRASP)',
    3: 'at_pos (reached target, no object = FAIL)',
}

# ---------------------------------------------------------------------------
# Colors
# ---------------------------------------------------------------------------
GREEN  = '\033[92m'
RED    = '\033[91m'
YELLOW = '\033[93m'
CYAN   = '\033[96m'
RESET  = '\033[0m'

logger = logging.getLogger('gripper_test')


class GripperTestNode(Node):
    """ROS2 node that exercises every AG-95 Modbus primitive."""

    def __init__(self):
        super().__init__('gripper_test_node')
        self.cb_group = ReentrantCallbackGroup()

        # --- service clients ---
        self._enable_cli = self.create_client(
            EnableRobot, f'{SRV_NS}/EnableRobot', callback_group=self.cb_group)
        self._power_cli = self.create_client(
            SetToolPower, f'{SRV_NS}/SetToolPower', callback_group=self.cb_group)
        self._create_cli = self.create_client(
            ModbusCreate, f'{SRV_NS}/ModbusCreate', callback_group=self.cb_group)
        self._set_cli = self.create_client(
            SetHoldRegs, f'{SRV_NS}/SetHoldRegs', callback_group=self.cb_group)
        self._get_cli = self.create_client(
            GetHoldRegs, f'{SRV_NS}/GetHoldRegs', callback_group=self.cb_group)
        self._close_modbus_cli = self.create_client(
            ModbusClose, f'{SRV_NS}/ModbusClose', callback_group=self.cb_group)

        # results tracking
        self._results: list[tuple[str, bool, str]] = []

    # ------------------------------------------------------------------
    # Service call helper — identical pattern to depalletizer.py:250-271
    # ------------------------------------------------------------------
    def _call_service(self, client, request, timeout=3.0):
        """Call a ROS2 service with a manual timeout loop.
        Returns the response object, or None on failure/timeout."""
        if not client.service_is_ready():
            self.get_logger().error(f'Service not ready: {client.srv_name}')
            return None
        future = client.call_async(request)
        t0 = time.time()
        while rclpy.ok() and not future.done() and (time.time() - t0) < timeout:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().error(f'Timeout calling {client.srv_name}')
            return None
        if future.exception() is not None:
            self.get_logger().error(
                f'Exception {client.srv_name}: {future.exception()}')
            return None
        resp = future.result()
        self.get_logger().debug(
            f'[Gripper] {client.srv_name} '
            f'res={getattr(resp, "res", None)} '
            f'robot_return="{getattr(resp, "robot_return", "")}"')
        return resp

    # ------------------------------------------------------------------
    # Register helpers — identical to depalletizer.py:273-309
    # ------------------------------------------------------------------
    def _set_reg(self, addr, value):
        """Write a U16 register. val_tab MUST have curly braces: '{value}'."""
        req = SetHoldRegs.Request()
        req.index    = MODBUS_INDEX
        req.addr     = addr
        req.count    = 1
        req.val_tab  = '{' + str(value) + '}'   # e.g. "{165}" — critical
        req.val_type = 'U16'
        resp = self._call_service(self._set_cli, req)
        time.sleep(INTER_CMD_DELAY)
        if resp is None or resp.res != 0:
            return False
        return True

    def _get_reg(self, addr):
        """Read a U16 register. Returns int or None on error."""
        req = GetHoldRegs.Request()
        req.index    = MODBUS_INDEX
        req.addr     = addr
        req.count    = 1
        req.val_type = 'U16'
        resp = self._call_service(self._get_cli, req)
        if resp is None or resp.res != 0:
            return None
        raw = resp.robot_return
        self.get_logger().debug(f'[Gripper] GetHoldReg addr={addr} raw="{raw}"')
        try:
            s = raw.strip().strip('{}')
            parts = [p.strip() for p in s.split(',') if p.strip()]
            return int(parts[0]) if parts else None
        except Exception as e:
            self.get_logger().error(
                f'Error parsing robot_return="{raw}": {e}')
            return None

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def _record(self, name, passed, detail=''):
        self._results.append((name, passed, detail))

    def _wait_for_services(self, timeout=5.0):
        """Wait for all service servers to be available."""
        clients = [
            self._enable_cli, self._power_cli, self._create_cli,
            self._set_cli, self._get_cli, self._close_modbus_cli,
        ]
        for cli in clients:
            if not cli.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(
                    f'Service {cli.srv_name} not available after {timeout}s')
                return False
        return True

    # ==================================================================
    # 12 TEST FUNCTIONS
    # ==================================================================

    def test_enable_robot(self):
        """Test 1: EnableRobot — verify res==0."""
        resp = self._call_service(self._enable_cli, EnableRobot.Request(), timeout=5.0)
        passed = resp is not None and resp.res == 0
        detail = f'res={getattr(resp, "res", None)}'
        self._record('test_enable_robot', passed, detail)
        return passed

    def test_set_tool_power(self):
        """Test 2: SetToolPower(status=1) — verify res==0."""
        req = SetToolPower.Request()
        req.status = 1
        resp = self._call_service(self._power_cli, req, timeout=5.0)
        passed = resp is not None and resp.res == 0
        detail = f'res={getattr(resp, "res", None)}'
        self._record('test_set_tool_power', passed, detail)
        return passed

    def test_modbus_create(self):
        """Test 3: ModbusCreate — verify res==0."""
        # Close any existing connection first
        req_close = ModbusClose.Request()
        req_close.index = 0
        self._call_service(self._close_modbus_cli, req_close, timeout=3.0)
        time.sleep(INTER_CMD_DELAY)

        req = ModbusCreate.Request()
        req.ip       = MODBUS_IP
        req.port     = MODBUS_PORT
        req.slave_id = MODBUS_SLAVE_ID
        req.is_rtu   = MODBUS_IS_RTU
        resp = self._call_service(self._create_cli, req, timeout=5.0)
        passed = resp is not None and resp.res == 0
        detail = f'res={getattr(resp, "res", None)}'
        self._record('test_modbus_create', passed, detail)
        return passed

    def test_gripper_init(self):
        """Test 4: Write 165 to reg 256, poll reg 512 until ==1 (15s timeout)."""
        ok = self._set_reg(REG_INIT, INIT_VALUE)
        if not ok:
            self._record('test_gripper_init', False, 'SetHoldReg(256,165) failed')
            return False

        t0 = time.time()
        while time.time() - t0 < INIT_TIMEOUT:
            val = self._get_reg(REG_INIT_STATUS)
            if val == 1:
                self._record('test_gripper_init', True, 'reg 512==1 (ready)')
                return True
            time.sleep(0.5)

        self._record('test_gripper_init', False,
                      f'Timeout {INIT_TIMEOUT}s waiting for reg 512==1')
        return False

    def test_set_force(self):
        """Test 5: Write 50 to reg 257 — verify res==0."""
        ok = self._set_reg(REG_FORCE, FORCE)
        self._record('test_set_force', ok,
                      f'reg 257={FORCE}' if ok else 'SetHoldReg(257) failed')
        return ok

    def test_set_speed(self):
        """Test 6: Write 50 to reg 258 — verify res==0."""
        ok = self._set_reg(REG_SPEED, SPEED)
        self._record('test_set_speed', ok,
                      f'reg 258={SPEED}' if ok else 'SetHoldReg(258) failed')
        return ok

    def test_gripper_open(self):
        """Test 7: Write 1000 to reg 259, poll reg 513 until !=0,
        read reg 514, assert actual_pos >= 950."""
        ok = self._set_reg(REG_POSITION, OPEN_POS)
        if not ok:
            self._record('test_gripper_open', False,
                          'SetHoldReg(259,1000) failed')
            return False

        t0 = time.time()
        while time.time() - t0 < POLL_TIMEOUT:
            status = self._get_reg(REG_STATUS)
            if status is not None and status != 0:
                actual = self._get_reg(REG_ACTUAL_POS)
                passed = actual is not None and actual >= 950
                detail = (f'status={status} ({STATUS_MAP.get(status, "?")}) '
                          f'actual_pos={actual}')
                self._record('test_gripper_open', passed, detail)
                return passed
            time.sleep(POLL_INTERVAL)

        self._record('test_gripper_open', False,
                      f'Timeout {POLL_TIMEOUT}s waiting for reg 513!=0')
        return False

    def test_gripper_close(self):
        """Test 8: Write 500 to reg 259, poll reg 513 until !=0,
        read reg 514, print status."""
        ok = self._set_reg(REG_POSITION, CLOSE_POS)
        if not ok:
            self._record('test_gripper_close', False,
                          'SetHoldReg(259,500) failed')
            return False

        t0 = time.time()
        while time.time() - t0 < POLL_TIMEOUT:
            status = self._get_reg(REG_STATUS)
            if status is not None and status != 0:
                actual = self._get_reg(REG_ACTUAL_POS)
                detail = (f'status={status} ({STATUS_MAP.get(status, "?")}) '
                          f'actual_pos={actual}')
                self._record('test_gripper_close', True, detail)
                return True
            time.sleep(POLL_INTERVAL)

        self._record('test_gripper_close', False,
                      f'Timeout {POLL_TIMEOUT}s waiting for reg 513!=0')
        return False

    def test_read_status(self):
        """Test 9: Read reg 513, print human-readable status."""
        val = self._get_reg(REG_STATUS)
        if val is None:
            self._record('test_read_status', False, 'GetHoldReg(513) failed')
            return False
        detail = f'reg 513={val} ({STATUS_MAP.get(val, "unknown")})'
        self._record('test_read_status', True, detail)
        return True

    def test_read_position(self):
        """Test 10: Read reg 514, print actual position."""
        val = self._get_reg(REG_ACTUAL_POS)
        if val is None:
            self._record('test_read_position', False,
                          'GetHoldReg(514) failed')
            return False
        detail = f'reg 514={val} (actual position)'
        self._record('test_read_position', True, detail)
        return True

    def test_full_cycle(self):
        """Test 11: Init -> open -> close -> read status -> open -> ModbusClose."""
        steps = [
            ('init',     self.test_gripper_init),
            ('open',     self.test_gripper_open),
            ('close',    self.test_gripper_close),
            ('status',   self.test_read_status),
            ('open2',    self.test_gripper_open),
            ('modbus_close', self.test_modbus_close),
        ]
        all_ok = True
        for label, fn in steps:
            print(f'  {CYAN}[full_cycle]{RESET} sub-step: {label}')
            if not fn():
                all_ok = False
                print(f'  {RED}[full_cycle]{RESET} sub-step {label} FAILED — aborting cycle')
                break
        # Record overall full_cycle result (individual sub-steps already recorded)
        self._record('test_full_cycle', all_ok,
                      'all sub-steps passed' if all_ok else 'aborted on failure')
        return all_ok

    def test_modbus_close(self):
        """Test 12: ModbusClose(index=0) — verify res==0."""
        req = ModbusClose.Request()
        req.index = MODBUS_INDEX
        resp = self._call_service(self._close_modbus_cli, req, timeout=3.0)
        passed = resp is not None and resp.res == 0
        detail = f'res={getattr(resp, "res", None)}'
        self._record('test_modbus_close', passed, detail)
        return passed

    # ==================================================================
    # Runner
    # ==================================================================

    # Map of --test flag values to (test_function, requires_setup)
    # requires_setup means tests 1-3 (enable/power/modbus) must run first
    TEST_MAP_ORDERED = [
        ('enable_robot',   'test_enable_robot'),
        ('set_tool_power', 'test_set_tool_power'),
        ('modbus_create',  'test_modbus_create'),
        ('gripper_init',   'test_gripper_init'),
        ('set_force',      'test_set_force'),
        ('set_speed',      'test_set_speed'),
        ('open',           'test_gripper_open'),
        ('close',          'test_gripper_close'),
        ('status',         'test_read_status'),
        ('position',       'test_read_position'),
        ('full_cycle',     'test_full_cycle'),
        ('modbus_close',   'test_modbus_close'),
    ]

    def run_tests(self, test_name=None):
        """Run all tests or a specific one. Returns (passed, total)."""
        self._results = []

        print(f'\n{CYAN}=== AG-95 Gripper Modbus Test Suite ==={RESET}\n')

        # Wait for services
        print(f'{YELLOW}Waiting for dobot_bringup services...{RESET}')
        if not self._wait_for_services(timeout=10.0):
            print(f'{RED}Services not available. Is dobot_bringup running?{RESET}')
            return 0, 1

        if test_name is not None:
            # Run single test
            method_name = None
            for key, mname in self.TEST_MAP_ORDERED:
                if key == test_name:
                    method_name = mname
                    break
            if method_name is None:
                print(f'{RED}Unknown test: {test_name}{RESET}')
                return 0, 1
            fn = getattr(self, method_name)
            print(f'[1/1] {method_name} ... ', end='', flush=True)
            fn()
        else:
            # Run all tests in order
            total = len(self.TEST_MAP_ORDERED)
            for i, (key, method_name) in enumerate(self.TEST_MAP_ORDERED, 1):
                fn = getattr(self, method_name)
                print(f'[{i}/{total}] {method_name} ... ', end='', flush=True)
                fn()
                # Print inline result for the test(s) just recorded
                # (full_cycle records multiple, so print only the latest)
                if self._results:
                    name, passed, detail = self._results[-1]
                    if passed:
                        print(f'{GREEN}PASS{RESET} ({detail})')
                    else:
                        print(f'{RED}FAIL{RESET} ({detail})')

        # If single-test mode, print the result now
        if test_name is not None and self._results:
            name, passed, detail = self._results[-1]
            if passed:
                print(f'{GREEN}PASS{RESET} ({detail})')
            else:
                print(f'{RED}FAIL{RESET} ({detail})')

        # Summary
        total = len(self._results)
        passed_count = sum(1 for _, p, _ in self._results if p)
        print(f'\n{CYAN}=== SUMMARY: {passed_count}/{total} tests passed ==={RESET}\n')

        # Detailed results
        for name, passed, detail in self._results:
            icon = f'{GREEN}PASS{RESET}' if passed else f'{RED}FAIL{RESET}'
            print(f'  {icon}  {name}: {detail}')

        return passed_count, total


def main():
    parser = argparse.ArgumentParser(
        description='Standalone AG-95 Gripper Modbus Test Suite for Dobot CR20')
    parser.add_argument(
        '--test',
        choices=[
            'enable_robot', 'set_tool_power', 'modbus_create',
            'gripper_init', 'set_force', 'set_speed',
            'open', 'close', 'status', 'position',
            'full_cycle', 'modbus_close',
        ],
        help='Run a specific test (default: run all)',
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s [%(levelname)s] %(message)s')

    rclpy.init()
    node = GripperTestNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # Spin executor in background thread so service futures get processed
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        passed, total = node.run_tests(test_name=args.test)
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
