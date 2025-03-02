cmake_minimum_required(VERSION 2.8.3)
project(mav_simulator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

cs_add_library(${PROJECT_NAME}
        src/mav_simulator.cpp
        )

cs_add_executable(${PROJECT_NAME}_node
        src/mav_simulator_node.cpp
        )
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} yaml-cpp)

cs_install()
cs_export()
