cmake_minimum_required(VERSION 3.22)
project(cr20_moveit)

find_package(ament_cmake REQUIRED)

# 1. Instalar SOLO las carpetas que existen en este paquete
install(DIRECTORY 
  launch 
  config 
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 2. Instalar los scripts de Python como ejecutables
install(PROGRAMS 
  scripts/cr20_isaac_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Instalar archivo de configuración si existe
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
    install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
