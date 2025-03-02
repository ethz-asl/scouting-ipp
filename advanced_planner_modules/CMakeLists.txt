cmake_minimum_required(VERSION 2.8.3)
project(planner_modules_planexp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

catkin_package()
find_package(ompl REQUIRED)
include_directories(
        ${OMPL_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME}
        src/map/map_planexp.cpp
        src/sensor_model/sensor_model_planexp.cpp
        src/sensor_model/camera_model_planexp.cpp
        src/cost_computer/cost_planexp_map_integral.cpp
        src/trajectory_evaluator/goal_distance_evaluator.cpp
        src/trajectory_evaluator/expand_low_cost_area_evaluator.cpp
        src/trajectory_evaluator/average_view_cost_evaluator.cpp
        src/trajectory_evaluator/expand_reachable_area_evaluator.cpp
        src/trajectory_evaluator/combined_evaluator.cpp
        src/trajectory_evaluator/rrt_star_evaluator.cpp
        src/evaluator_updater/rrt_star_updater.cpp
        src/evaluator_updater/update_all_non_terminal.cpp
        src/initialization/planexp_package.cpp
        )
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} stdc++fs)

cs_install()
cs_export()