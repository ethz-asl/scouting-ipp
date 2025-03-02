cmake_minimum_required(VERSION 2.8.3)
project(ros_planner_planexp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

catkin_package()

cs_add_library(${PROJECT_NAME}
        src/ros_planner_planexp.cpp
        )

cs_add_executable(ros_planner_planexp_node
        src/ros_planner_planexp_node.cpp
        )
target_link_libraries(ros_planner_planexp_node ${PROJECT_NAME} ${catkin_LIBRARIES} stdc++fs)

cs_install()
cs_export()
