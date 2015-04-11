cmake_minimum_required(VERSION 2.8)
include(FRCDeploy)

project(MyRobot)
find_package(WPILib)
include_directories(${WPILib_INCLUDE_DIRS})

file(GLOB_RECURSE SOURCE_CODE *.cpp)

add_executable(out ${SOURCE_CODE})
target_link_libraries(out ${WPILib_LIBRARY})

add_make_deploy(out 10.6.91.2) # where your team number is xxyy
