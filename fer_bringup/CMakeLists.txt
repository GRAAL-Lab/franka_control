cmake_minimum_required(VERSION 3.16)
project(fer_bringup)
option(BUILD_TESTING "Build tests" OFF)
# Accept the workspace-wide option; hardware targets are defined in fer_driver.
option(FER_BUILD_HARDWARE "Build the libfranka hardware executables" ON)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch conf DESTINATION share/${PROJECT_NAME})
ament_package()
