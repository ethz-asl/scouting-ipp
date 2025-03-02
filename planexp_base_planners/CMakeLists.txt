cmake_minimum_required(VERSION 2.8.12)
project(planexp_base_planners)

add_definitions(-std=c++17)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        roslib
        tf
        eigen_catkin
        cv_bridge
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp rospy
        LIBRARIES ${PROJECT_NAME}
)

find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

find_package(ompl)


#############
# LIBRARIES #
#############
include_directories(
        include
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${YamlCpp_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/prm_star_planner.cpp
        src/rrt_star_planner.cpp
        src/rrt_star_ompl_base.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES} stdc++fs)


################
# EVAL PLANNER #
################
add_executable(planexp_base_planner_node
        src/planexp_base_planner_node.cpp
        )
add_dependencies(planexp_base_planner_node ${PROJECT_NAME})
target_link_libraries(planexp_base_planner_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} pthread)

################
# EVAL PLANNER #
################
add_executable(planexp_eval_planner
        src/planexp_eval_planner.cpp
        )
add_dependencies(planexp_eval_planner ${PROJECT_NAME})
target_link_libraries(planexp_eval_planner ${PROJECT_NAME} ${catkin_LIBRARIES} ${YamlCpp_LIBRARIES} pthread)