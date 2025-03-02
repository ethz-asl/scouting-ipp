#!/usr/bin/env python

# Python
import sys
import time
import csv
import datetime
import os
import re
import subprocess

# ros
import rospy
from planexp_msgs.msg import MapData
from planexp_msgs.msg import PlannerStatus
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from planexp_msgs.srv import FilePath
from planexp_msgs.srv import PathCost
from planexp_msgs.srv import ExploredArea
from planexp_msgs.srv import ExplorableArea


class EvalData(object):
    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_planner = rospy.get_param('~ns_planner',
                                          "/firefly/planner_node")
        self.planner_delay = rospy.get_param(
            '~delay', 0.0)  # Waiting time until the planner is launched
        self.evaluate = rospy.get_param(
            '~evaluate', False)  # Periodically save the voxblox state
        self.startup_timeout = rospy.get_param(
            '~startup_timeout', 0.0)  # Max allowed time for startup, 0 for inf

        self.eval_frequency = rospy.get_param('~eval_frequency',
                                              5.0)  # Save rate in seconds
        self.time_limit = rospy.get_param(
            '~time_limit', 0.0)  # Maximum sim duration in minutes, 0 for inf
        # self.reset_unreal_cv_ros = rospy.get_param(
        #     '~reset_unreal_cv_ros', True)  # On shutdown reset pose to 0
        # self.ns_unreal_cv_ros = rospy.get_param('~ns_unreal_cv_ros',
        #                                         "/unreal/unreal_ros_client")
        self.ns_mav = rospy.get_param('~ns_mav',
                                          "/firefly")
        self.abort_on_collision = rospy.get_param('~abort_on_collision', True)

        self.is_baseline = rospy.get_param('~is_baseline', False)

        self.eval_walltime_0 = None
        self.eval_rostime_0 = None

        if self.evaluate:
            # Setup parameters
            self.eval_directory = rospy.get_param(
                '~eval_directory',
                'DirParamNotSet')  # Periodically save voxblox map
            if not os.path.isdir(self.eval_directory):
                rospy.logfatal("Invalid target directory '%s'.",
                               self.eval_directory)
                sys.exit(-1)

            self.ns_map = rospy.get_param('~ns_map_server',
                                              "/voxblox/voxblox_node") #TODO: change to map space

            # Statistics
            self.eval_n_maps = 0
            self.eval_n_map_data = 0
            self.eval_n_map_data_acc = 0

            # Setup data directory
            if not os.path.isdir(os.path.join(self.eval_directory,
                                              "tmp_bags")):
                os.mkdir(os.path.join(self.eval_directory, "tmp_bags"))
            '''self.eval_directory = os.path.join(
                self.eval_directory,
                datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))'''
            #os.mkdir(self.eval_directory)
            rospy.set_param(self.ns_planner + "/performance_log_dir",
                            self.eval_directory)
            os.mkdir(os.path.join(self.eval_directory, "maps"))
            self.eval_data_file = open(
                os.path.join(self.eval_directory, "map_data.csv"), 'w')
            self.eval_writer = csv.writer(self.eval_data_file,
                                          delimiter=',',
                                          quotechar='|',
                                          quoting=csv.QUOTE_MINIMAL,
                                          lineterminator='\n')
            self.eval_writer.writerow(
                ['MapName', 'RosTime', 'WallTime', 'NMapData', 'CPUTime', 'TravelCost', 'CostType', 'ExplorableArea', 'ExploredArea', 'GoalExplored'])
            self.eval_writer.writerow(
                ['Unit', 'seconds', 'seconds', '-', 'seconds', '-', '-', 'm^2', 'm^2', '-'])
            self.eval_log_file = open(
                os.path.join(self.eval_directory, "data_log.txt"), 'a')

            # Subscribers, Services
            self.ue_out_sub = rospy.Subscriber(self.ns_mav + "/map_data",
                                               MapData,
                                               self.map_data_callback,
                                               queue_size=10)
            self.collision_sub = rospy.Subscriber(self.ns_mav + "/collision",
                                                  String,
                                                  self.collision_callback,
                                                  queue_size=10)
            self.cpu_time_srv = rospy.ServiceProxy(
                self.ns_planner + "/get_cpu_time", SetBool)

            self.eval_cost_service = rospy.ServiceProxy(self.ns_mav + "/path_cost", PathCost)
            self.eval_goal_explored_service = rospy.ServiceProxy(self.ns_planner + "/goal_explored", Trigger)
            self.goal_explored = False
            self.eval_explorable_area_service = rospy.ServiceProxy(self.ns_mav + "/explorable_area", ExplorableArea)
            self.explorable_area = 0.0
            self.eval_explored_area_service = rospy.ServiceProxy(
                self.ns_mav + "/explored_area", ExploredArea)

            self.goal_requested_sub = rospy.Subscriber(self.ns_mav + "/goal_location_requested",
                                                       Empty,
                                                       self.goal_requested_callback,
                                                       queue_size=10)
            self.map_planner_status_sub = rospy.Subscriber(self.ns_planner + "/planner_status",
                                                       PlannerStatus,
                                                       self.map_planner_status_callback,
                                                       queue_size=100)

            # Finish
            self.writelog("Data folder created at '%s'." % self.eval_directory)
            rospy.loginfo("Data folder created at '%s'." % self.eval_directory)
            self.eval_map_service = rospy.ServiceProxy(
                self.ns_planner + "/save_map", FilePath)
            rospy.on_shutdown(self.eval_finish)
            self.collided = False

        self.launch_simulation()

    def launch_simulation(self):
        rospy.loginfo(
            "Experiment setup: waiting for unreal MAV simulation to setup...")
        # Wait for unreal simulation to setup
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_message("unreal_simulation_ready", String,
                                       self.startup_timeout) #TODO: remove or replace
            except rospy.ROSException:
                self.stop_experiment(
                    "Simulation startup failed (timeout after " +
                    str(self.startup_timeout) + "s).")
                return
        else:
            pass
            #rospy.wait_for_message("unreal_simulation_ready", String)
        rospy.loginfo("Waiting for unreal MAV simulation to setup... done.")

        # Launch planner (by service, every planner needs to advertise this
        # service when ready)
        rospy.loginfo("Waiting for planner to be ready...")
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_service(self.ns_planner + "/toggle_running",
                                       self.startup_timeout)
            except rospy.ROSException:
                self.stop_experiment("Planner startup failed (timeout after " +
                                     str(self.startup_timeout) + "s).")
                return
        else:
            rospy.wait_for_service(self.ns_planner + "/toggle_running")

        if self.planner_delay > 0:
            rospy.loginfo(
                "Waiting for planner to be ready... done. Launch in %d "
                "seconds.", self.planner_delay)
            rospy.sleep(self.planner_delay)
        else:
            rospy.loginfo("Waiting for planner to be ready... done.")

        self.explorable_area = self.eval_explorable_area_service()
        self.explorable_area = self.explorable_area.explorable_area
        set_initial_position_srv = rospy.ServiceProxy(
            self.ns_mav + "/set_initial_position", Trigger)
        initial_position_set = set_initial_position_srv()
        if initial_position_set.success:
            rospy.loginfo(initial_position_set.message)
            if self.evaluate:
                self.writelog(initial_position_set.message)
        else:
            self.stop_experiment(initial_position_set.message)
            return
        run_planner_srv = rospy.ServiceProxy(
            self.ns_planner + "/toggle_running", SetBool)
        run_planner_srv(True)

        # Setup first measurements
        self.eval_walltime_0 = time.time()
        self.eval_rostime_0 = rospy.get_time()
        # Evaluation init
        if self.evaluate:
            self.writelog("Succesfully started the simulation.")

            # Dump complete rosparams for reference
            subprocess.check_call([
                "rosparam", "dump",
                os.path.join(self.eval_directory, "rosparams.yaml"), "/"
            ])
            self.writelog("Dumped the parameter server into 'rosparams.yaml'.")

            self.eval_n_maps = 0
            self.eval_n_map_data = 1

            # Keep track of the (most recent) rosbag
            bag_expr = re.compile(
                r'tmp_bag_\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}\.bag.'
            )  # Default names
            bags = [
                b for b in os.listdir(
                    os.path.join(self.eval_directory,
                                 "tmp_bags")) if bag_expr.match(b)
            ]
            bags.sort(reverse=True)
            if bags:
                self.writelog("Registered '%s' as bag for this simulation." %
                              bags[0])
                self.eval_log_file.write("[FLAG] Rosbag: %s\n" %
                                         bags[0].split('.')[0])
            else:
                rospy.logwarn("No tmpbag found. Is rosbag recording?")

        # Periodic evaluation (call once for initial measurement)
        self.eval_callback(None)
        rospy.Timer(rospy.Duration(self.eval_frequency), self.eval_callback)

        # Finish
        rospy.loginfo("\n" + "*" * 39 +
                      "\n* Succesfully started the simulation! *\n" + "*" * 39)

    def eval_callback(self, _):
        if self.evaluate:
            # Produce a data point
            time_real = time.time() - self.eval_walltime_0
            time_ros = rospy.get_time() - self.eval_rostime_0
            map_name = "{0:05d}".format(self.eval_n_maps)
            try:
                cpu = self.cpu_time_srv(True)
            except:
                # Usually this means the planner died
                self.stop_experiment("Planner Node died (cpu srv failed).")
                return
            try:
                travel_cost_msg = self.eval_cost_service(self.eval_n_map_data_acc-1) #TODO: find out if the -1 is correct here
                travel_cost = travel_cost_msg.cost
                cost_type = travel_cost_msg.cost_type
            except:
                travel_cost = -1
                cost_type = "unknown"
            try:
                goal_explored = self.eval_goal_explored_service()
                goal_explored = goal_explored.success
                self.goal_explored = (self.goal_explored or goal_explored)
            except:
                goal_explored = False
            try:
                explored_area = self.eval_explored_area_service(self.eval_n_map_data_acc-1)
                explored_area = explored_area.explored_area
            except:
                explored_area = -1

            self.eval_writer.writerow([
                map_name, time_ros, time_real, self.eval_n_map_data,
                float(cpu.message), travel_cost, cost_type, self.explorable_area, explored_area, goal_explored
            ])
            self.eval_map_service(
                os.path.join(self.eval_directory, "maps", map_name))
            self.eval_n_map_data = 0
            self.eval_n_maps += 1

        # If the time limit is reached stop the simulation
        if self.time_limit > 0.0:
            if rospy.get_time(
            ) - self.eval_rostime_0 >= self.time_limit * 60.0:
                self.stop_experiment("Time limit reached.")
        # If the map is fully explored stop the simulation
        if self.explorable_area > 0:
            if (explored_area / self.explorable_area) >= 0.99 and self.goal_explored:
                self.stop_experiment("Map fully explored. (99% + Goal)")
        else:
            if self.goal_explored:
                self.stop_experiment("Map fully explored. (Goal)")

    def eval_finish(self):
        self.eval_data_file.close()
        map_path = os.path.join(self.eval_directory, "maps")
        n_maps = len([
            f for f in os.listdir(map_path)
            if (os.path.isfile(os.path.join(map_path, f)) and ("cost.png" in f))
        ])
        self.writelog("Finished the simulation, %d/%d maps created." %
                      (n_maps, self.eval_n_maps))
        self.eval_log_file.close()
        rospy.loginfo("On eval_data_node shutdown: closing data files.")

    def writelog(self, text):
        # In case of simulation data being stored, maintain a log file
        if not self.evaluate:
            return
        self.eval_log_file.write(
            datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S] ") + text +
            "\n")

    def map_data_callback(self, _): #TODO: check what this is or should be doing
        if self.evaluate:
            self.eval_n_map_data += 1
            self.eval_n_map_data_acc += 1

    def stop_experiment(self, reason):
        # Shutdown the node with proper logging, only required when experiment
        # is performed
        reason = "Stopping the experiment: " + reason
        if self.evaluate:
            self.writelog(reason)
        '''if self.reset_unreal_cv_ros: #TODO: Implement propper shutdown and remove unreal cv maybe later replace with oaisys
            try:
                # If unreal is running, this will reset it, otherwise map is
                # already in initial state
                terminate_srv = rospy.ServiceProxy(
                    self.ns_unreal_cv_ros + "/terminate_with_reset", SetBool)
                terminate_srv(True)
            except:
                pass'''
        width = len(reason) + 4
        rospy.loginfo("\n" + "*" * width + "\n* " + reason + " *\n" +
                      "*" * width)
        rospy.signal_shutdown(reason)

    def collision_callback(self, msg):
        if self.abort_on_collision:
            if not self.collided:
                self.writelog(msg.data)
                self.collided = True
                self.stop_experiment("Collision detected!")
        else:
            self.writelog(msg.data)
            self.collided = True

    def goal_requested_callback(self, _):
        self.writelog("Goal requested callback was called.")
        self.eval_callback(self)
        if self.is_baseline:
            self.stop_experiment("Goal reached!")

    def map_planner_status_callback(self, msg):
        if msg.status == PlannerStatus.STATUS_TERMINAL_INVALID:
            self.eval_callback(self)
            self.stop_experiment("Map planner failed and can not recover anymore!")
        elif msg.status == PlannerStatus.STATUS_TERMINAL_VALID:
            self.eval_callback(self)
            self.stop_experiment("Terminal solution found!")
        elif msg.status == PlannerStatus.STATUS_INVALID:
            self.writelog("Encountered INVALID map planner status.")
        elif msg.status == PlannerStatus.STATUS_VALID:
            pass
        elif msg.status == PlannerStatus.STATUS_UNKNOWN:
            self.writelog("Encountered UNKNOWN map planner status.")
        else:
            self.writelog("Encountered undefined map planner status.")



if __name__ == '__main__':
    rospy.init_node('eval_data_node', anonymous=True)
    ed = EvalData()
    rospy.spin()
