cmake_minimum_required(VERSION 3.16)
project(fer_tcp_bridge)
option(BUILD_TESTING "Build tests" OFF)
# Accept the workspace-wide option; hardware targets are defined in fer_driver.
option(FER_BUILD_HARDWARE "Build the libfranka hardware executables" ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fer_msgs REQUIRED)
find_package(fer_model REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(fer_tcp_bridge_lib
    src/fer_tcp_bridge/arm_command_parser.cpp
    src/fer_tcp_bridge/franka_tcp_bridge.cpp
)
target_compile_features(fer_tcp_bridge_lib PUBLIC cxx_std_17)
target_include_directories(fer_tcp_bridge_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(fer_tcp_bridge_lib PUBLIC Boost::system fer_model::fer_model)
ament_target_dependencies(fer_tcp_bridge_lib PUBLIC rclcpp fer_msgs)

add_executable(tcp_bridge_node src/tcp_bridge_node.cpp)
target_link_libraries(tcp_bridge_node fer_tcp_bridge_lib)
add_executable(dummy_client_node src/dummy_client.cpp)
target_link_libraries(dummy_client_node fer_tcp_bridge_lib)
foreach(target fer_tcp_bridge_lib tcp_bridge_node dummy_client_node)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS fer_tcp_bridge_lib EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS tcp_bridge_node dummy_client_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp fer_msgs fer_model Boost)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_arm_command_parser test/test_arm_command_parser.cpp)
    target_link_libraries(test_arm_command_parser fer_tcp_bridge_lib)
endif()
ament_package(CONFIG_EXTRAS cmake/fer_tcp_bridge-extras.cmake)
