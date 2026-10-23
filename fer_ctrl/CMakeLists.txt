cmake_minimum_required(VERSION 3.16)
project(fer_ctrl)

option(BUILD_TESTING "Build tests" OFF)
# Accept the workspace-wide option; hardware targets are defined in fer_driver.
option(FER_BUILD_HARDWARE "Build the libfranka hardware executables" ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fer_msgs REQUIRED)
find_package(fer_model REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rml REQUIRED)
find_package(fsm REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(tpik REQUIRED)
find_package(ikcl REQUIRED)

add_library(fer_dynamic_ctrl_lib
    src/fer_ctrl/dynamic_controller.cpp
)
target_compile_features(fer_dynamic_ctrl_lib PUBLIC cxx_std_17)
target_include_directories(fer_dynamic_ctrl_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(fer_dynamic_ctrl_lib PUBLIC fer_model::fer_model config++)

add_library(fer_kinematic_ctrl_lib
    src/fer_ctrl/configuration.cpp
    src/fer_ctrl/task_priority_controller.cpp
    src/fer_ctrl/kinematic_controller.cpp
    src/fer_ctrl/tasks/joints_limit.cpp
    src/fer_ctrl/tasks/obstacle_avoidance.cpp
    src/fer_ctrl/states/generic_state.cpp
    src/fer_ctrl/states/state_disabled.cpp
    src/fer_ctrl/states/state_disabling.cpp
    src/fer_ctrl/states/state_enabling.cpp
    src/fer_ctrl/states/state_fault.cpp
    src/fer_ctrl/states/state_idle.cpp
    src/fer_ctrl/states/state_parked.cpp
    src/fer_ctrl/states/state_parking.cpp
    src/fer_ctrl/states/state_unparking.cpp
    src/fer_ctrl/states/state_move_joints_pos.cpp
    src/fer_ctrl/states/state_move_joints_vel.cpp
    src/fer_ctrl/states/state_move_cartesian.cpp
    src/fer_ctrl/commands/generic_command.cpp
    src/fer_ctrl/commands/command_disable.cpp
    src/fer_ctrl/commands/command_enable.cpp
    src/fer_ctrl/commands/command_stop.cpp
    src/fer_ctrl/commands/command_park.cpp
    src/fer_ctrl/commands/command_unpark.cpp
    src/fer_ctrl/commands/command_move_joints_pos.cpp
    src/fer_ctrl/commands/command_move_joints_vel.cpp
    src/fer_ctrl/commands/command_move_cartesian.cpp
)
target_compile_features(fer_kinematic_ctrl_lib PUBLIC cxx_std_17)
target_include_directories(fer_kinematic_ctrl_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(fer_kinematic_ctrl_lib PUBLIC
    fer_model::fer_model rml fsm ctrl_toolbox tpik ikcl config++
)
ament_target_dependencies(fer_kinematic_ctrl_lib PUBLIC rclcpp fer_msgs geometry_msgs tf2_ros ament_index_cpp)

add_executable(kinematic_control_node src/kinematic_control_node.cpp)
target_link_libraries(kinematic_control_node fer_kinematic_ctrl_lib)

add_executable(dynamic_control_node
    src/dynamic_control_node.cpp
    src/fer_ctrl/dynamic_control_node.cpp
)
target_link_libraries(dynamic_control_node fer_dynamic_ctrl_lib)
ament_target_dependencies(dynamic_control_node rclcpp fer_msgs ament_index_cpp)

foreach(target fer_dynamic_ctrl_lib fer_kinematic_ctrl_lib dynamic_control_node kinematic_control_node)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()

install(TARGETS fer_kinematic_ctrl_lib fer_dynamic_ctrl_lib
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(TARGETS kinematic_control_node dynamic_control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY conf launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fer_model fer_msgs rclcpp geometry_msgs tf2_ros ament_index_cpp rml fsm ctrl_toolbox tpik ikcl)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_controller test/test_controller.cpp)
    target_link_libraries(test_controller fer_kinematic_ctrl_lib)
    target_compile_definitions(test_controller PRIVATE
        FER_KCL_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/conf/kcl_fer.conf"
    )
    ament_add_gtest(test_dynamic_controller test/test_dynamic_controller.cpp)
    target_link_libraries(test_dynamic_controller fer_dynamic_ctrl_lib)
    target_compile_definitions(test_dynamic_controller PRIVATE
        FER_DCL_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/conf/dcl_fer.conf"
    )
endif()

ament_package()
