cmake_minimum_required(VERSION 3.8)
project(tesi_code)

if(NOT CMAKE_C_STANDARD)   
  set(CMAKE_C_STANDARD 99)   
endif()   

if(NOT CMAKE_CXX_STANDARD)   
  set(CMAKE_CXX_STANDARD 14)   
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")   
  add_compile_options(-Wall -Wextra -Wpedantic)   
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/record_odom_data.py
  scripts/calculate_path.py
  scripts/record_data.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  ign_world
  DESTINATION share/${PROJECT_NAME})
  

ament_package()
