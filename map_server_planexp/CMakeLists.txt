cmake_minimum_required(VERSION 2.8.3)
project(map_server_planexp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

catkin_package()
include_directories(
        include/${PROJECT_NAME}
        ${OMPL_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME}
        src/map_server_planexp.cpp
        )

cs_add_executable(${PROJECT_NAME}_node
        src/map_server_planexp_node.cpp
        )
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} yaml-cpp ${OMPL_LIBRARIES} ${catkin_LIBRARIES} stdc++fs)

cs_install()
cs_export()
