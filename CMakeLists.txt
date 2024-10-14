cmake_minimum_required(VERSION 3.8)
project(ros_tcp_endpoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME}
  cpp/src/main.cpp   
  cpp/src/tcpServerNode.cpp
  cpp/src/unityTcpSender.cpp
  cpp/src/clientThread.cpp
  cpp/src/rosSerialization.cpp
  cpp/src/rosCommunication.cpp
  cpp/src/rosPublisher.cpp
  cpp/src/rosSubscriber.cpp
  cpp/src/rosService.cpp
  cpp/src/unityService.cpp
  cpp/src/statusEvent.cpp
  cpp/src/threadPauser.cpp
  cpp/src/sysCommands.cpp
  cpp/src/generic_service.cpp
  )
ament_target_dependencies(${PROJECT_NAME} rclcpp)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
    $<INSTALL_INTERFACE:include>
  )
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(${PROJECT_NAME} PUBLIC NOMINMAX) # WIN32_LEAN_AND_MEAN

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
