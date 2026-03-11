#!/usr/bin/env python3
"""
Test script for Apriltag 36h11 detection using RealSense camera
"""
import pyrealsense2 as rs
import numpy as np
import pupil_apriltags
import cv2
import time

def main():
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable color stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    profile = pipeline.start(config)
    
    # Get camera intrinsics
    color_profile = profile.get_stream(rs.stream.color)
    intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
    
    # Print camera intrinsics
    print(f"Camera intrinsics:")
    print(f"  fx: {intrinsics.fx}")
    print(f"  fy: {intrinsics.fy}")
    print(f"  cx: {intrinsics.ppx}")
    print(f"  cy: {intrinsics.ppy}")
    
    # Create Apriltag detector
    # tag_family="tag36h11" is the standard tag family
    # tag_size in meters (typical is around 0.166 for tag36h11)
    detector = pupil_apriltags.Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )
    
    print("\nDetecting Apriltag 36h11 markers...")
    print("Press 'q' to quit\n")
    
    try:
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                continue
            
            # Convert to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            
            # Camera parameters for detection
            camera_params = (
                intrinsics.fx,  # fx
                intrinsics.fy,  # fy
                intrinsics.ppx, # cx
                intrinsics.ppy  # cy
            )
            
            # Detect tags
            tags = detector.detect(
                gray,
                estimate_tag_pose=True,
                tag_size=0.166,  # 36h11 standard size in meters
                camera_params=camera_params
            )
            
            # Draw results on image
            for tag in tags:
                # Draw tag border
                pts = tag.corners.astype(np.int32)
                cv2.polylines(color_image, [pts], True, (0, 255, 0), 2)
                
                # Draw tag ID
                cx = int(tag.center[0])
                cy = int(tag.center[1])
                cv2.putText(
                    color_image, 
                    f"ID: {tag.tag_id}",
                    (cx - 30, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )
                
                # Print detection info
                print(f"Tag detected: ID={tag.tag_id}, "
                      f"center=({tag.center[0]:.1f}, {tag.center[1]:.1f}), "
                      f"pose={tag.pose_t}")
            
            # Show image
            cv2.imshow('Apriltag Detection', color_image)
            
            # Quit on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
