cmake_minimum_required(VERSION 2.8.3)
project(planexp_msgs)

find_package(catkin_simple REQUIRED message_generation std_msgs geometry_msgs trajectory_msgs eigen_catkin)
catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########

cs_install()
cs_export()