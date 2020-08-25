#!/usr/bin/env python3

import os
import subprocess
import sys
import threading
import time
import rospy
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib
from gi.repository import Pango

import yaml

import std_msgs.msg
from std_msgs.msg import Bool

from autoware_config_msgs.msg import ConfigNDT
from autoware_config_msgs.msg import ConfigVoxelGridFilter
from autoware_config_msgs.msg import ConfigWaypointFollower
from autoware_config_msgs.msg import ConfigTwistFilter
from autoware_config_msgs.msg import ConfigDecisionMaker
from autoware_config_msgs.msg import ConfigRayGroundFilter

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2

from autoware_msgs.msg import ControlCommandStamped
from autoware_msgs.msg import Lane
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import BrakeCmd
from autoware_msgs.msg import IndicatorCmd
from autoware_msgs.msg import LampCmd
from autoware_msgs.msg import TrafficLight
from autoware_msgs.msg import DetectedObjectArray

nodes = [["Detection", ["LiDAR detector", False], ["camera detector", False], ["lidar_kf_contour_track", False], 
            ["lidar camera fusion", False], ["lidar_euclidean_cluster_detect", False]],
         ["Follower", ["twist filter", False], ["pure pursuit", False], ["mpc", False], ["hybride stenly", False]],
         ["Localizer", ["ndt matching", False], ["ekf localizer", False]],
         ["Decision Maker", ["decision maker", False]],
         ["LaneChange Manager", ["lanechange manager", False]],
         ["Local Planner", ["op_common_params", False], ["op_trajectory_generator", False], ["op_motion_predictor", False],
            ["op_trajectory_evaluator", False],["op_behavior_selector", False]],
         ["Vehicle Setting", ["vel_pose_connect", False], ["baselink to localizer", False]],
         ["QUICK START", ["detection_quick_start", False],["planning_quick_start", False]]]

kill_instruction = [["/obb_generator /qt_detect_node", "/vision_darknet_detect /yolo3_rects",  "/lidar_kf_contour_track", 
                        "/detection/fusion_tools/range_fusion_visualization_01 /range_vision_fusion_01", "/detection/lidar_detector/cluster_detect_visualization_01 /lidar_euclidean_cluster_detect"],
               ["/twist_filter /twist_gate", "/pure_pursuit", "/mpc_follower /mpc_waypoints_converter", "/hybride_stenly"],
               ["/ndt_matching", "/ekf_localizer"],
               ["/decision_maker"],
               ["/lanechange_manager"],
               ["/op_common_params", "/op_trajectory_generator", "/op_motion_predictor", "/op_trajectory_evaluator", "/op_behavior_selector"],
               ["/pose_relay /vel_relay", "/base_link_to_localizer"]]

map_nodes = [["Map", ["point cloud", False], ["vector map", False], ["point_vector tf", False]],
             ["Sensing", ["ray_ground_filter", False], ["cloud_transformer", False], ["sensor3", False], ["sensor4", False]],
             ["Point Downsampler", ["voxel grid filter", False]],
             ["QUICK START", ["map_quick_start", False], ["sensing_quick_start", False]]]

kill_instruction2 = [["/points_map_loader", "/vector_map_loader", "/world_to_map"],
                ["/ray_ground_filter", "/cloud_transformer", "/sensor3", "/sensor4"],
                ["/voxel_grid_filter"]]

node_sequence_list = []
inst_sequence_list = []

check_alive = [["", "", "", "", ""], # Detection
               ["", ""], # Follower
               ["", ""], # Localizer
               [""], # Decision maker
               [""], # LaneChange manager
               ["", "", "", ""], # Local Planner
               [""], # Vehicle Setting
               ["", "", ""], # Map
               ["", "", "", ""], # Sensing
               [""]] # Point Downsampler

alive_names = [["LiDAR detector", "camera detector", "lidar_kf_contour_track", "lidar camera fusion", "lidar_euclidean_cluster_detect"], # Detection
               ["twist filter", "waypoint_follower"], # Follower
               ["ndt matching", "ekf localizer"], # Localizer
               ["decision maker"], # Decision maker
               ["lanechange manager"], # LaneChange manager
               ["op_motion_predictor", "op_trajectory_evaluator", "op_trajectory_generator", "op_behavior_selector"], # Local Planner
               ["vel_pose_connect"], # Vehicle Setting
               ["point cloud", "vector map", "point_vector tf"], # Map
               ["ray_ground_filter", "cloud_transformer", "sensor3", "sensor4"], # Sensing
               ["voxel_grid_filter"]] # Point Downsampler

estop_state = False

def getDirPath():
	return os.path.abspath(os.path.dirname(__file__)) + "/"

def load_yaml(filename):
    dir_path = getDirPath()
    path = dir_path + filename
    print('loading... ' + filename)
    f = open(path, 'r')
    d = yaml.load(f)
    f.close()
    return d

def getYamlIndex(config, name):
    for list_ in config:
        if name in list_:
            return list_[name]

default_yaml = load_yaml('default_param.yaml')
planning_quickstart_yaml = load_yaml('planning_quickstart.yaml')
detection_quickstart_yaml = load_yaml('detection_quickstart.yaml')
sensing_quickstart_yaml = load_yaml('sensing_quickstart.yaml')
map_quickstart_yaml = load_yaml('map_quickstart.yaml')

class AutowareConfigPublisher:
    def __init__(self):
        self.pub_twist_filter       = rospy.Publisher('/config/twist_filter', ConfigTwistFilter, latch=True, queue_size=10)
        self.pub_waypoint_follower  = rospy.Publisher('/config/waypoint_follower', ConfigWaypointFollower, latch=True, queue_size=10)
        self.pub_ndt                = rospy.Publisher('/config/ndt', ConfigNDT, latch=True, queue_size=10)
        self.pub_decision_maker     = rospy.Publisher('/config/decision_maker', ConfigDecisionMaker, latch=True, queue_size=10)
        self.pub_voxel_grid_filter  = rospy.Publisher('/config/voxel_grid_filter', ConfigVoxelGridFilter, latch=True, queue_size=10)
        self.pub_ray_ground_filter  = rospy.Publisher('/config/ray_ground_filter', ConfigRayGroundFilter, latch=True, queue_size=10)
        
        self.setDefaultConfigParam()

    def setDefaultConfigParam(self):
        
        config_ = default_yaml.get('config', [])
        setup_ = default_yaml.get('setup', [])
        for inst in setup_:
            for cmd in inst['tf_setup']:
                print(cmd['cmd'])
                os.system(cmd['cmd'])

        val = getYamlIndex(config_, 'conf_twist_filter')
        data = ConfigTwistFilter()
        data.lateral_accel_limit    = float(val[0]['lateral_accel_limit'])
        data.lowpass_gain_linear_x  = float(val[1]['lowpass_gain_linear_x'])
        data.lowpass_gain_angular_z = float(val[2]['lowpass_gain_angular_z'])
        self.onConfigTwistFilter(data)

        val = getYamlIndex(config_, 'conf_waypoint_follower')
        data = ConfigWaypointFollower()
        data.param_flag                 = int(val[0]['param_flag'])
        data.velocity                   = float(val[1]['velocity'])
        data.lookahead_distance         = float(val[2]['lookahead_distance'])
        data.lookahead_ratio            = float(val[3]['lookahead_ratio'])
        data.minimum_lookahead_distance = float(val[4]['minimum_lookahead_distance'])
        data.displacement_threshold     = float(val[5]['displacement_threshold'])
        data.relative_angle_threshold   = float(val[6]['relative_angle_threshold'])
        self.onConfigWaypointFollower(data)

        val = getYamlIndex(config_, 'conf_ndt')
        data = ConfigNDT()
        data.init_pos_gnss    = int(val[0]['init_pos_gnss'])
        data.use_predict_pose = int(val[1]['use_predict_pose'])
        data.error_threshold  = float(val[2]['error_threshold'])
        data.resolution       = float(val[3]['resolution'])
        data.step_size        = float(val[4]['step_size'])
        data.trans_epsilon    = float(val[5]['trans_epsilon'])
        data.max_iterations   = int(val[6]['max_iterations'])
        self.onConfigNdt(data)

        val = getYamlIndex(config_, 'conf_decision_maker')
        data = ConfigDecisionMaker()
        data.auto_mission_reload    = bool( val[0]['auto_mission_reload'])
        data.auto_engage            = bool( val[1]['auto_engage'])
        data.auto_mission_change    = bool( val[2]['auto_mission_change'])
        data.use_fms                = bool( val[3]['use_fms'])
        data.disuse_vector_map      = bool( val[4]['disuse_vector_map'])
        data.num_of_steer_behind    = int(  val[5]['num_of_steer_behind'])
        data.change_threshold_dist  = float(val[6]['change_threshold_dist'])
        data.change_threshold_angle = float(val[7]['change_threshold_angle'])
        data.goal_threshold_dist    = float(val[8]['goal_threshold_dist'])
        data.goal_threshold_vel     = float(val[9]['goal_threshold_vel'])
        data.stopped_vel            = float(val[10]['stopped_vel'])
        self.onConfigDecisionMaker(data)

        val = getYamlIndex(config_, 'conf_voxel_grid_filter')
        data = ConfigVoxelGridFilter()
        data.voxel_leaf_size = float(val[0]['voxel_leaf_size'])
        data.measurement_range = float(val[1]['measurement_range'])
        self.onConfigVoxelGridFilter(data)

        val = getYamlIndex(config_, 'conf_ray_ground_filter')
        data = ConfigRayGroundFilter()
        data.sensor_height               = float(val[0]['sensor_height'])
        data.clipping_height             = float(val[1]['clipping_height'])
        data.min_point_distance          = float(val[2]['min_point_distance'])
        data.radial_divider_angle        = float(val[3]['radial_divider_angle'])
        data.concentric_divider_distance = float(val[4]['concentric_divider_distance'])
        data.local_max_slope             = float(val[5]['local_max_slope'])
        data.general_max_slope           = float(val[6]['general_max_slope'])
        data.min_height_threshold        = float(val[7]['min_height_threshold'])
        data.reclass_distance_threshold  = float(val[8]['reclass_distance_threshold'])
        self.onConfigRayGroundFilter(data)

    def onConfigTwistFilter(self, data): # twist filter
        self.pub_twist_filter.publish(data)

    def onConfigWaypointFollower(self, data): # pure pursuit
        self.pub_waypoint_follower.publish(data)

    def onConfigNdt(self, data): # ndt matching
        self.pub_ndt.publish(data)

    def onConfigDecisionMaker(self, data): # decision maker
        self.pub_decision_maker.publish(data)

    def onConfigVoxelGridFilter(self, data): # voxel grid filter
        self.pub_voxel_grid_filter.publish(data)

    def onConfigRayGroundFilter(self, data): # ray_ground_filter
        self.pub_ray_ground_filter.publish(data)

class AutowareAliveNodesCheck:
    def __init__(self):
        rospy.Subscriber('/detection/lidar_detector/objects', DetectedObjectArray, self.checkLidarDetect) # lidar detect
        rospy.Subscriber('/detection/image_detector/objects', DetectedObjectArray, self.checkCameraDetect) # camera detect
        rospy.Subscriber('/detection/fusion_tools/objects', DetectedObjectArray, self.checkRangeVisionFusion) # range vision fusion
        rospy.Subscriber('tracked_objects', DetectedObjectArray, self.checkKfContourTrack) # kf contour
        rospy.Subscriber('twist_cmd', TwistStamped, self.checkTwistFilter) # twist filter
        rospy.Subscriber('/ctrl_cmd', ControlCommandStamped, self.checkNdtMatching) # ndt
        rospy.Subscriber('/ndt_pose', PoseStamped, self.checkWaypointFollower) # pure pursuit or mpc or hybride stanley
        rospy.Subscriber('/ekf_ndt_pose', PoseStamped, self.checkEkfLocalizer) # ekf localizer
        rospy.Subscriber('/decision_maker/state', std_msgs.msg.String, self.checkDecisionMaker) # decision maker
        rospy.Subscriber('lanechange_check', std_msgs.msg.String, self.checkLaneChangeManager) # lanechange manager
        rospy.Subscriber('/predicted_objects', DetectedObjectArray, self.checkOpMotionPredictor) # op motion predict
        rospy.Subscriber('local_trajectory_cost', Lane, self.checkOpTrajectoryEval) # op trajectory evaluator
        rospy.Subscriber('local_trajectories', LaneArray, self.checkOpTrajectoryGen) # op trajectory generator
        rospy.Subscriber('final_waypoints', Lane, self.checkOpBehaviorSelector) # op behavior selector
        rospy.Subscriber('/current_velocity', TwistStamped, self.checkVelPoseConnect) # vel pose connect
        rospy.Subscriber('points_map', PointCloud2, self.checkPointMapLoader) # point map
        rospy.Subscriber('vmap_stat', Bool, self.checkVectorMapLoader) # vector map
        rospy.Subscriber('/filtered_points', PointCloud2, self.checkVoxelGridFilter) # voxel grid filter
        rospy.Subscriber('/points_no_ground', PointCloud2, self.checkRayGroundFilter) # ray ground filter
        rospy.Subscriber('/points_transformed', PointCloud2, self.checkCloudTransformer) # cloud transformer
        
        # rospy.Subscriber('sensor topic', msgtype, checkCallBack) # sensor check subscriber

        self.deadCount = [[-1,-1,-1,-1,-1], # Detection
                          [-1,-1], # Follower
                          [-1,-1], # Localizer
                          [-1], # Decision maker
                          [-1], # LaneChange manager
                          [-1,-1,-1,-1], # Local Planner
                          [-1], # Vehicle Setting
                          [-1,-1,-1], # Map
                          [-1,-1,-1,-1], # Sensing
                          [-1]] # Point Downsampler

        self.mutex = threading.Lock()

        self.update_thread = threading.Thread(target=self.upDateAliveState)
        self.update_thread.daemon = True
        self.update_thread.start()
        self.reset_time = 3

    def changeAliveState(self):
        self.checkWorldToMap()
        if "/vector_map_loader" not in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[7][1] = -1
            self.mutex.release()
        
        if "/points_map_loader" not in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[7][0] = -1
            self.mutex.release()

        if estop_state == True:
            for i in range(len(self.deadCount)):
                for j in range(len(self.deadCount[i])):
                    self.deadCount[i][j] = -1

        for i in range(len(self.deadCount)):
            for j in range(len(self.deadCount[i])):
                if self.deadCount[i][j] < 0:
                    check_alive[i][j] = "X"
                else:
                    check_alive[i][j] = "True"
    def checkLidarDetect(self, data):
        if "/obb_generator /qt_detect_node" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[0][0]   = self.reset_time
            self.mutex.release()
    def checkCameraDetect(self, data):
        if "/yolo3_rects" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[0][1]   = self.reset_time
            self.mutex.release()
    def checkRangeVisionFusion(self, data):
        if "/lidar_kf_contour_track" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[0][2]   = self.reset_time
            self.mutex.release()
    def checkKfContourTrack(self, data):
        if "/detection/fusion_tools/range_fusion_visualization_01 /range_vision_fusion_01" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[0][3]   = self.reset_time
            self.mutex.release()
    def checkTwistFilter(self, data):
        if "/twist_filter /twist_gate" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[1][0]   = self.reset_time
            self.mutex.release()
    def checkWaypointFollower(self, data):
        self.mutex.acquire()
        self.deadCount[1][1]   = self.reset_time
        self.mutex.release()
    def checkNdtMatching(self, data):
        if "/ndt_matching" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[2][0]   = self.reset_time
            self.mutex.release()
    def checkEkfLocalizer(self, data):
        if "/ekf_localizer" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[2][1]   = self.reset_time
            self.mutex.release()
    def checkDecisionMaker(self, data):
        if "/decision_maker" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[3][0]   = self.reset_time
            self.mutex.release()
    def checkLaneChangeManager(self, data):
        if "/lanechange_manager" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[4][0]   = self.reset_time
            self.mutex.release()
    def checkOpMotionPredictor(self, data):
        if "/op_motion_predictor" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[5][0]   = self.reset_time
            self.mutex.release()
    def checkOpTrajectoryEval(self, data):
        if "/op_trajectory_evaluator" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[5][1]   = self.reset_time
            self.mutex.release()
    def checkOpTrajectoryGen(self, data):
        if "/op_trajectory_generator" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[5][2]   = self.reset_time
            self.mutex.release()
    def checkOpBehaviorSelector(self, data):
        if "/op_behavior_selector" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[5][3]   = self.reset_time
            self.mutex.release()
    def checkVelPoseConnect(self, data):
        if "/pose_relay /vel_relay" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[6][0]   = self.reset_time
            self.mutex.release()
    def checkPointMapLoader(self, data):
        if "/points_map_loader" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[7][0]   = 999999
            self.mutex.release()
    def checkVectorMapLoader(self, data):
        if "/vector_map_loader" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[7][1]   = 999999
            self.mutex.release()
    def checkWorldToMap(self):
        if "/world_to_map" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[7][2]   = self.reset_time
            self.mutex.release()
    #def checkSensor(self, data):
    #    if "sensor node name" in node_sequence_list:
    #        self.mutex.acquire()
    #        self.deadCount[8][]   = self.reset_time
    #        self.mutex.release()
    def checkRayGroundFilter(self, data):
        if "/ray_ground_filter" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[8][0]   = self.reset_time
            self.mutex.release()
    def checkCloudTransformer(self, data):
        if "/cloud_transformer" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[8][1]   = self.reset_time
            self.mutex.release()
    def checkVoxelGridFilter(self, data):
        if "/voxel_grid_filter" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[9][0]   = self.reset_time
            self.mutex.release()

    def upDateAliveState(self):
        while 1:
            time.sleep(1)
            self.mutex.acquire()
            for i in range(len(self.deadCount)):
                for j in range(len(self.deadCount[i])):
                    self.deadCount[i][j] = self.deadCount[i][j] - 1
            self.mutex.release()

class MyWindow(Gtk.ApplicationWindow):
    def __init__(self, app):
        rospy.init_node('ichthus_manager', anonymous=False)

        Gtk.Window.__init__(self, title="Ichthus manager", application=app)
        self.set_default_size(800, 600)
        self.set_border_width(20)

        autoware_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.node_tree = Gtk.TreeStore(str, bool)
        # fill in the model
        for i in range(len(nodes)):
            parent_iter = self.node_tree.append(None, [nodes[i][0], False])
            j = 1
            while j < len(nodes[i]):
                self.node_tree.append(parent_iter, nodes[i][j])
                j += 1
                 
        self.node_tree2 =  Gtk.TreeStore(str, bool)
        for i in range(len(map_nodes)):
            parent_iter2 = self.node_tree2.append(None, [map_nodes[i][0], False])
            j = 1
            while j < len(map_nodes[i]):
                self.node_tree2.append(parent_iter2, map_nodes[i][j])
                j += 1
        
        # generate autoware checking class  
        self.config_pub = AutowareConfigPublisher()
        self.alive_node_check = AutowareAliveNodesCheck()
        
        # the treeview shows the model
        # create a treeview on the model self.node_tree
        self.nodes_Lwin = Gtk.ScrolledWindow()
        self.nodes_Lwin.set_border_width(2)
        self.nodes_Lwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.nodes_Rwin = Gtk.ScrolledWindow()
        self.nodes_Rwin.set_border_width(2)
        self.nodes_Rwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        view = Gtk.TreeView()
        autoware_box.pack_start(self.nodes_Lwin, True, True, 0)
        self.nodes_Lwin.add(view)
        view.set_model(self.node_tree)

        # the cellrenderer for the first column - text
        renderer_nodes = Gtk.CellRendererText()
        # the first column is created
        column_nodes1 = Gtk.TreeViewColumn("Computing", renderer_nodes, text=0)
        
        # and it is appended to the treeview
        view.append_column(column_nodes1)
        
        # the cellrenderer for the second column - boolean rendered as a toggle
        renderer_in_out = Gtk.CellRendererToggle()
        # the second column is created
        column_in_out = Gtk.TreeViewColumn("Execute", renderer_in_out, active=1)
        # and it is appended to the treeview
        view.append_column(column_in_out)

        # connect the cellrenderertoggle with a callback function
        renderer_in_out.connect("toggled", self.onToggled)

        view2 = Gtk.TreeView()
        autoware_box.pack_start(self.nodes_Rwin, True, True, 0)
        self.nodes_Rwin.add(view2)
        view2.set_model(self.node_tree2)
        renderer_nodes2 = Gtk.CellRendererText()
        column_nodes2 = Gtk.TreeViewColumn("Map & Sensing", renderer_nodes2, text=0)
        view2.append_column(column_nodes2)
        renderer_in_out2 = Gtk.CellRendererToggle()
        column_in_out2 = Gtk.TreeViewColumn("Execute", renderer_in_out2, active=1)
        view2.append_column(column_in_out2)
        renderer_in_out2.connect("toggled", self.onToggled2)
        
        # Resource Check Frame code start
        resource_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        cpu_frame = Gtk.Frame()
        self.mem_frame = Gtk.Frame()
        self.disk_frame = Gtk.Frame()
        resource_box.pack_start(cpu_frame, True, True, 0)
        
        cpu_frame.set_shadow_type(1)
        cpu_frame.set_border_width(2)
        cpu_frame.set_label("cpu resource")

        self.mem_frame.set_shadow_type(1)
        self.mem_frame.set_border_width(2)
        self.mem_frame.set_label("memory resource")

        self.disk_frame.set_shadow_type(1)
        self.disk_frame.set_border_width(2)
        self.disk_frame.set_label("disk resource")

        self.mem_disk_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        resource_box.pack_start(self.mem_disk_box, True, True, 0)

        self.mem_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.disk_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.cpu_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)

        self.mem_disk_box.pack_start(self.mem_frame, True, True, 0)
        self.mem_disk_box.pack_start(self.disk_frame, True, True, 0)

        self.mem_label = Gtk.Label('mem check')
        self.mem_box.pack_start(self.mem_label, True, True, 0)
        self.mem_frame.add(self.mem_box)

        self.disk_label = Gtk.Label('disk check')
        self.disk_box.pack_start(self.disk_label, True, True, 0)
        self.disk_frame.add(self.disk_box)
        
        self.cpubar = []
        self.cpu_persentage = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cpu_cnt = int(os.cpu_count())
        i = 0
        while 1:
            if i == cpu_cnt:
                break
            self.cpubar.append(Gtk.ProgressBar())
            i = i + 1

        i = 0
        while 1:
            if i == cpu_cnt:
                break
            self.cpu_box.pack_start(self.cpubar[i], True, True, 0)
            i = i + 1
        cpu_frame.add(self.cpu_box)

        self.timeout_id = GLib.timeout_add(50, self.onTimeOut, None)
        # Resource Check Frame code end 
        
        # Parametor stack setting code start
        parameter_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.alive_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        
        self.alive_label = Gtk.Label('Select Component')
        self.alive_box.pack_start(self.alive_label, True, True, 0)

        self.param_Lframe = Gtk.Frame()
        self.param_Rframe = Gtk.Frame()
        self.exception_frame = Gtk.Frame()

        self.param_Lframe.set_shadow_type(1)
        self.param_Lframe.set_border_width(2)
        self.param_Lframe.set_label("alive nodes")

        self.param_Rframe.set_shadow_type(1)
        self.param_Rframe.set_border_width(2)
        self.param_Rframe.set_label("alive list")

        self.alive_list_text = Gtk.TextView()
        self.alive_list_text.set_editable(False)

        self.exception_frame.set_shadow_type(1)
        self.exception_frame.set_border_width(2)
        self.exception_frame.set_label("exception buttons")

        self.param_Lwin = Gtk.ScrolledWindow()
        self.param_Lwin.set_border_width(2)
        self.param_Lwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        self.param_Rwin = Gtk.ScrolledWindow()
        self.param_Rwin.set_border_width(2)
        self.param_Rwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        self.alive_nodes_grid = Gtk.Grid()
        self.alive_nodes_grid.set_column_homogeneous(True)
        self.alive_nodes_grid.set_row_homogeneous(True)
        self.param_Lframe.add(self.alive_nodes_grid)
        self.alive_nodes_grid.attach(self.alive_box, 0, 0, 1, 13)
 
        self.textview_grid = Gtk.Grid()
        self.textview_grid.set_column_homogeneous(True)
        self.textview_grid.set_row_homogeneous(True)
        self.param_Rframe.add(self.alive_list_text)
        self.textview_grid.attach(self.param_Rframe, 0, 0, 1, 4)

        estop_button= Gtk.Button.new_with_label("E-STOP")
        rerun_button= Gtk.Button.new_with_label("Re Run")

        estop_button.connect("clicked", self.doSystemEstop)
        rerun_button.connect("clicked", self.doRerunRoutine)

        self.except_grid = Gtk.Grid()
        self.except_grid.set_column_homogeneous(True)
        self.except_grid.set_row_homogeneous(True)
        self.except_grid.attach(estop_button, 0, 0, 1, 1)
        self.exception_frame.add(self.except_grid)

        component_store = Gtk.ListStore(int, str)
        component_store.append([0, "Detection"])
        component_store.append([1, "Follower"])
        component_store.append([2, "Localizer"])
        component_store.append([3, "Decision Maker"])
        component_store.append([4, "LaneChange Manager"])
        component_store.append([5, "Local Planner"])
        component_store.append([6, "Vehicle Setting"])
        component_store.append([7, "Map"])
        component_store.append([8, "Sensing"])
        component_store.append([9, "Point Downsampler"])
        component_store.append([10, "Show All"])

        self.alive_view_idx = len(component_store) - 1

        component_combo = Gtk.ComboBox.new_with_model_and_entry(component_store)
        component_combo.connect("changed", self.onComponentComboChanged)
        component_combo.set_entry_text_column(1)

        self.alive_nodes_grid.attach_next_to(
            component_combo, self.alive_box, Gtk.PositionType.BOTTOM, 1, 1
        )
        self.textview_grid.attach_next_to(
            self.exception_frame, self.param_Rframe, Gtk.PositionType.BOTTOM, 1, 1
        )
        self.except_grid.attach_next_to(
            rerun_button, estop_button, Gtk.PositionType.BOTTOM, 1, 1
        )

        self.param_Lwin.add(self.param_Lframe)
        self.param_Rwin.add(self.textview_grid)

        parameter_box.pack_start(self.param_Lwin, True, True, 0)
        parameter_box.pack_start(self.param_Rwin, True, True, 0)
        # Parametor stack setting code end.

        self.mem_thread = threading.Thread(target=self.setText, args=(self.mem_box,))
        self.mem_thread.daemon = True
        self.mem_thread.start()

        self.cpu_thread = threading.Thread(target=self.setCpuText, args=(self.cpu_box,))
        self.cpu_thread.daemon = True
        self.cpu_thread.start()
        
        self.disk_thread = threading.Thread(target=self.getDistSpace, args=(self.disk_box,))
        self.disk_thread.daemon = True
        self.disk_thread.start()

        self.update_alive_list_thread = threading.Thread(target=self.setAlivelist, args=(self.alive_list_text,))
        self.update_alive_list_thread.daemon = True
        self.update_alive_list_thread.start()

        stack = Gtk.Stack()
        stack.add_titled(autoware_box, 'child1', 'Autoware Nodes')  
        stack.add_titled(parameter_box, 'child2', 'Alive Check')  
        stack.add_titled(resource_box, 'child3', 'Resource Check')  

        stack_switcher = Gtk.StackSwitcher(stack=stack) 

        header_bar = Gtk.HeaderBar(custom_title=stack_switcher, show_close_button=True) 

        
        self.set_titlebar(header_bar) 
        self.add(stack)

   #     rospy.Subscriber('/exception_estop', Bool, self.recvSystemEstop) # estop
   #     rospy.Subscriber('/exception_rerun', Bool, self.recvRerunRoutine) # rerun
   # def recvSystemEstop(self, data):
   #     self.doSystemEstop()
   # def recvRerunRoutine(self, data):
   #     self.doRerunRoutine()

    def setAlivelist(self, widget):
        while 1:
            self.alive_node_check.changeAliveState()
            self.m_list_buff = Gtk.TextBuffer()
            time.sleep(1)
            st = ''
            for idx1 in range(len(check_alive)):
                for idx2 in range(len(check_alive[idx1])):
                    if check_alive[idx1][idx2] == "True":
                        st = st + alive_names[idx1][idx2] + '\n'
            #print(st)
            self.m_list_buff.set_text(st)
            widget.set_buffer(self.m_list_buff)
            tmp = self.alive_list_text.get_buffer()
            self.alive_list_text.show()

    def onComponentComboChanged(self, combo):
        tree_iter = combo.get_active_iter()
        # self.alive_node_check.changeAliveState()
        if tree_iter is not None:
            model = combo.get_model()
            row_id, name = model[tree_iter][:2]
            #print("Selected: ID=%d, name=%s" % (row_id, name))
            self.alive_view_idx = row_id

            detect_txt = ('Detection : \n' 
                            + '     LiDAR detector ---------------> ' + check_alive[0][0] + '\n'
                            + '     camera detector -------------> '  + check_alive[0][1] + '\n'
                            + '     lidar_kf_contour_track --> '      + check_alive[0][2] + '\n'
                            + '     lidar camera fusion --------> '   + check_alive[0][3] + '\n\n')
            follower_txt = ('Follower : \n' 
                            + '     twist filter ---------------> '   + check_alive[1][0] + '\n'
                            + '     follower ------------> '          + check_alive[1][1] + '\n\n')
            localizer_txt = ('Localizer : \n' 
                            + '     ndt matching --------> '          + check_alive[2][0] + '\n'
                            + '     ekf localizer -----------> '      + check_alive[2][1] + '\n\n')
            decision_txt = ('Decision Maker : \n'
                            + '     decision maker --------->'        + check_alive[3][0] + '\n\n')
            lanechange_txt = ('LaneChange Manager : \n' 
                            + '     lanechange manager ------> '      + check_alive[4][0] + '\n\n')
            local_plan_txt = ('Local Planner : \n' 
                            + '     op_motion_predictor --------> '   + check_alive[5][0] + '\n'
                            + '     op_trajectory_evaluator ---> '    + check_alive[5][1] + '\n'
                            + '     op_trajectory_generator --> '     + check_alive[5][2] + '\n'
                            + '     op_behavior_selector -------> '   + check_alive[5][3] + '\n\n')
            vehicle_txt = ('Vehicle Setting : \n' 
                            + '     vel_pose_connect --------> '      + check_alive[6][0] + '\n\n')
            map_txt = ('Map : \n' 
                            + '     point cloud -------------> '      + check_alive[7][0] + '\n'
                            + '     vector map -------------> '       + check_alive[7][1] + '\n'
                            + '     point_vector tf -------> '       + check_alive[7][2] + '\n\n')
            sensing_txt = ('Sensing : \n'
                            + '     ray_ground_filter -------> ' + check_alive[8][0] + '\n'
                            + '     cloud_transformer -----> ' + check_alive[8][1] + '\n'
                            + '     sensor3 -------> ' + check_alive[8][2] + '\n'
                            + '     sensor4 -------> ' + check_alive[8][3] + '\n\n')
            downsampler_txt = ('Point Downsampler : \n'
                            + '     voxel grid filter -----> ' + check_alive[9][0] + '\n\n')

            if self.alive_view_idx == 0:
                txt = detect_txt
            elif self.alive_view_idx == 1:
                txt = follower_txt
            elif self.alive_view_idx == 2:
                txt = localizer_txt
            elif self.alive_view_idx == 3:
                txt = decision_txt
            elif self.alive_view_idx == 4:
                txt = lanechange_txt
            elif self.alive_view_idx == 5:
                txt = local_plan_txt
            elif self.alive_view_idx == 6:
                txt = vehicle_txt
            elif self.alive_view_idx == 7:
                txt = map_txt
            elif self.alive_view_idx == 8:
                txt = sensing_txt
            elif self.alive_view_idx == 9:
                txt = downsampler_txt
            elif self.alive_view_idx == 10:
                txt = (detect_txt + follower_txt + localizer_txt + decision_txt + 
                      lanechange_txt + local_plan_txt + vehicle_txt + map_txt + sensing_txt + downsampler_txt)
            
            free = self.alive_label.get_text()
            self.alive_label.set_text(txt)
        else:
            entry = combo.get_child()
            print("Entered: %s" % entry.get_text())

    def onTimeOut(self, user_data):
        """
        Update CPU resource value on the progress bar
        """
        cpu_cnt = int(os.cpu_count())
        i = 0
        while 1:
            if i == cpu_cnt:
                break
            self.cpubar[i].set_fraction(self.cpu_persentage[i] / 100)
            self.cpubar[i].set_text('CPU ' + str(i) + ' : ' + str(round(self.cpu_persentage[i], 2)) + ' %')
            self.cpubar[i].set_show_text(True)
            i = i + 1

        # As this is a timeout function, return True so that it
        # continues to get called
        return True

    def setCpuText(self, widget):
        cpu_core_count = int(os.cpu_count())

        while 1:
            p = os.popen('top -d 1 -n 2')
            i = 0
            j = 0
            while 1:
                line = p.readline()
                cur = line.split('\x1b(B\x1b[m\x1b[39;49m\x1b[1m')[0:5]
                #print(len(cur))
                if len(cur) != 0:
                    st = cur[0].split('\x1b(B\x1b[m\x1b[39;49m')[0]
                if st == 'Tasks:':
                    #print(st)
                    i = i + 1
                if i == 2:
                    line = p.readline()
                    #print(line)
                    cur = line.split('\x1b(B\x1b[m\x1b[39;49m\x1b[1m')[0:5]
                    if len(cur) < 5:
                        break
                    st = cur[4].split('\x1b(B\x1b[m\x1b[39;49m')[0]
                    #print(cur[4].split('\x1b(B\x1b[m\x1b[39;49m')[0])
                    self.cpu_persentage[0] = 100.0 - float(st)
                    while j < (cpu_core_count - 1):
                        j = j+1
                        line = p.readline()
                        #print(line)
                        cur = line.split('\x1b(B\x1b[m\x1b[39;49m\x1b[1m')[0:5]
                        if len(cur) < 5:
                            break
                        st = cur[4].split('\x1b(B\x1b[m\x1b[39;49m')[0]
                        #print(cur[0].split('\x1b(B\x1b[m\x1b[39;49m')[0])
                        used_cpu = 100.0 - float(st)
                        self.cpu_persentage[j] = used_cpu
                        #print(line)
                    break
    
    def setText(self, widget):
        total = 0
        while 1:
            p = os.popen('free')
            total = 0
            while 1:
                total = total + 1
                line = p.readline()
                if total==2:
                    sp_line = line.split()[1:4]
                    used_per = (float(sp_line[1]) / float(sp_line[0])) * 100
                    txt = 'TOTAL MEMORY :  ' + sp_line[0] + ' KB\n' + \
                          'USED  MEMORY  :  ' + sp_line[1] + ' KB\n' + \
                          'FREE  MEMORY   :  ' + sp_line[2] + ' KB\n' + \
                          'USED  MEM(%)    :  ' + str(round(used_per,2)) + '%'

                    break

            self.mem_label.set_text(txt)
            time.sleep(1)

    def getDistSpace(self, widget):
        while 1:
            p = os.popen("df -h /")
            i = 0
            while 1:
                i = i + 1
                line = p.readline()
                if i==2:
                    cur_disk = line.split()[1:5]
                    txt = 'TOTAL  SPACE      :  ' + cur_disk[0] + '\n' + \
                          'USED   SPACE       :  ' + cur_disk[1] + '\n' + \
                          'REMAIN SPACE   :  ' + cur_disk[2] + '\n' + \
                          'USED DISK (%)     :  ' + cur_disk[3]
                    break
            self.disk_label.set_text(txt)
            time.sleep(1)
            
    def getMemResource(self, widget):
        p = os.popen('free -h')
        i = 0
        while 1:
            i = i + 1
            line = p.readline()
            if i==2:
                widget.pack_start(Gtk.Label(line), True, True, 0)

    def getParamSettingWin(self, idx1, idx2):
        setparam_win = Gtk.Window()
        setparam_win.set_title("set params")
        setparam_win.set_default_size(300, 400)
        setparam_win.set_border_width(20)
        grid = Gtk.Grid()
        grid.set_column_homogeneous(True)
        grid.set_row_homogeneous(True)
        
        setparam_win.add(grid)

        set_param_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        grid.attach(set_param_box, 0, 0, 5, 10)

        in_grid = Gtk.Grid()
        in_grid.set_column_homogeneous(True)
        in_grid.set_row_homogeneous(True)
        exec_button = Gtk.Button.new_with_label("Execute")

        grid.attach_next_to(
            exec_button, set_param_box, Gtk.PositionType.BOTTOM, 1, 1
        )
        set_param_box.pack_start(in_grid, True, True, 0)

        launch_ = default_yaml.get('launch_param', [])
        config_ = default_yaml.get('config', [])

        param_list = []
        if idx1 == 0: # Detection
            if idx2 == 0: # lidar detector
                param1 = Gtk.Label('detect_range (25, 50)')
                param2 = Gtk.Label('segmentation rate')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()

                val = getYamlIndex(launch_, 'lidar_detector')

                #default values
                pv1.set_text(str(val[0]['detect_range']))
                pv2.set_text(str(val[1]['segmentation_rate']))
                
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)

                param_list = [pv1,pv2]
            # elif idx2 == 1: # camera detector
            elif idx2 == 2: # lidar kf contour track
                # lauch param
                param1 = Gtk.Label('tracking_type (Associate Only : 0, Simple Tracker : 1, Contour Tracker : 2)')
                param2 = Gtk.Label('min_object_size (0.0 ~ 2.0)')
                param3 = Gtk.Label('max_object_size (0.0 ~ 100.0)')
                param4 = Gtk.Label('vector_map_filter_distance (0.0 ~ 20.0)')
                param5 = Gtk.Label('enableLogging')
                param6 = Gtk.Label('polygon_quarters (4 ~ 32)')
                param7 = Gtk.Label('polygon_resolution (0.25 ~ 5.0)')
                param8 = Gtk.Label('max_association_distance (0.5 ~ 10.0)')
                param9 = Gtk.Label('max_association_size_diff (0.0 ~ 30.0)')
                param10 = Gtk.Label('max_remeber_time (0.0 ~ 10.0)')
                param11 = Gtk.Label('trust_counter (1 ~ 15)')
                param12 = Gtk.Label('enableSimulationMode')
                param13 = Gtk.Label('enableStepByStepMode')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()
                pv10 = Gtk.Entry()
                pv11 = Gtk.Entry()
                pv12 = Gtk.Entry()
                pv13 = Gtk.Entry()

                #print(launch_.index('lidar_kf_contour_track'))
                
                val = getYamlIndex(launch_, 'lidar_kf_contour_track')

                #default values
                pv1.set_text(str(val[0]['tracking_type'])) # tracking_type
                pv2.set_text(str(val[1]['min_object_size'])) # min_object_size
                pv3.set_text(str(val[2]['max_object_size'])) # max_object_size
                pv4.set_text(str(val[3]['vector_map_filter_distance'])) # vector_map_filter_distance
                pv5.set_text(str(val[4]['enableLogging'])) # enableLogging
                pv6.set_text(str(val[5]['polygon_quarters'])) # polygon_quarters
                pv7.set_text(str(val[6]['polygon_resolution'])) # polygon_resolution
                pv8.set_text(str(val[7]['max_association_distance'])) # max_association_distance
                pv9.set_text(str(val[8]['max_association_size_diff'])) # max_association_size_diff
                pv10.set_text(str(val[9]['max_remeber_time'])) # max_remeber_time
                pv11.set_text(str(val[10]['trust_counter'])) # trust_counter
                pv12.set_text(str(val[11]['enableSimulationMode'])) # enableSimulationMode
                pv13.set_text(str(val[12]['enableStepByStepMode'])) # enableStepByStepMode

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param10, param9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param11, param10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param12, param11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param13, param12, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv10, pv9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv11, pv10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv12, pv11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv13, pv12, Gtk.PositionType.BOTTOM, 1, 1)                

                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9,pv10,pv11,pv12,pv13]
#            elif idx2 == 3: # lidar camera fusion
            elif idx2 == 4: # lidar detector
                param1 = Gtk.Label('use_gpu')
                param2 = Gtk.Label('output_frame')
                param3 = Gtk.Label('pose_estimation')
                param4 = Gtk.Label('downsample_cloud')
                param5 = Gtk.Label('points_node')
                param6 = Gtk.Label('leaf_size (0.0 ~ 1.0)')
                param7 = Gtk.Label('cluster_size_min (1 ~ 100,000)')
                param8 = Gtk.Label('cluster_size_max (1 ~ 200,000)')
                param9 = Gtk.Label('clustering_distance (0.0 ~ 10.0)')
                param10 = Gtk.Label('clip_min_height (-5 ~ 5)')
                param11 = Gtk.Label('clip_max_height (0.0 ~ 5.0)')
                param12 = Gtk.Label('use_vector_map')
                param13 = Gtk.Label('vectormap_frame')
                param14 = Gtk.Label('wayarea_gridmap_topic')
                param15 = Gtk.Label('wayarea_gridmap_layer')
                param16 = Gtk.Label('wayarea_no_road_value (0 ~ 255)')
                param17 = Gtk.Label('remove_points_upto (0.0 ~ 2.5)')
                param18 = Gtk.Label('keep_lanes')
                param19 = Gtk.Label('keep_lane_left_distance (0.0 ~ 100.0)')
                param20 = Gtk.Label('keep_lane_right_distance (0.0 ~ 100.0)')
                param21 = Gtk.Label('cluster_merge_threshold (0.0 ~ 10.0)')
                param22 = Gtk.Label('use_multiple_thres')
                param23 = Gtk.Label('clustering_ranges')
                param24 = Gtk.Label('clustering_distances')
                param25 = Gtk.Label('remove_ground')
                param26 = Gtk.Label('use_diffnormals')
                param27 = Gtk.Label('publish_filtered')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()
                pv10 = Gtk.Entry()
                pv11 = Gtk.Entry()
                pv12 = Gtk.Entry()
                pv13 = Gtk.Entry()
                pv14 = Gtk.Entry()
                pv15 = Gtk.Entry()
                pv16 = Gtk.Entry()
                pv17 = Gtk.Entry()
                pv18 = Gtk.Entry()
                pv19 = Gtk.Entry()
                pv20 = Gtk.Entry()
                pv21 = Gtk.Entry()
                pv22 = Gtk.Entry()
                pv23 = Gtk.Entry()
                pv24 = Gtk.Entry()
                pv25 = Gtk.Entry()
                pv26 = Gtk.Entry()
                pv27 = Gtk.Entry()

                val = getYamlIndex(launch_, 'lidar_euclidean_cluster_detect')

                #default values
                pv1.set_text(str(val[0]['use_gpu'])) # use_gpu
                pv2.set_text(str(val[1]['output_frame'])) # output_frame
                pv3.set_text(str(val[2]['pose_estimation'])) # pose_estimation
                pv4.set_text(str(val[3]['downsample_cloud'])) # downsample_cloud
                pv5.set_text(str(val[4]['points_node'])) # points_node
                pv6.set_text(str(val[5]['leaf_size'])) # leaf_size
                pv7.set_text(str(val[6]['cluster_size_min'])) # cluster_size_min
                pv8.set_text(str(val[7]['cluster_size_max'])) # cluster_size_max
                pv9.set_text(str(val[8]['clustering_distance'])) # clustering_distance
                pv10.set_text(str(val[9]['clip_min_height'])) # clip_min_height
                pv11.set_text(str(val[10]['clip_max_height'])) # clip_max_height
                pv12.set_text(str(val[11]['use_vector_map'])) # use_vector_map
                pv13.set_text(str(val[12]['vectormap_frame'])) #  vectormap_frame
                pv14.set_text(str(val[13]['wayarea_gridmap_topic'])) # wayarea_gridmap_topic
                pv15.set_text(str(val[14]['wayarea_gridmap_layer'])) # wayarea_gridmap_layer
                pv16.set_text(str(val[15]['wayarea_no_road_value'])) # wayarea_no_road_value 
                pv17.set_text(str(val[16]['remove_points_upto'])) # remove_points_upto 
                pv18.set_text(str(val[17]['keep_lanes'])) # keep_lanes
                pv19.set_text(str(val[18]['keep_lane_left_distance'])) # keep_lane_left_distance
                pv20.set_text(str(val[19]['keep_lane_right_distance'])) # keep_lane_right_distance 
                pv21.set_text(str(val[20]['cluster_merge_threshold'])) # cluster_merge_threshold 
                pv22.set_text(str(val[21]['use_multiple_thres'])) #use_multiple_thres 
                pv23.set_text(str(val[22]['clustering_ranges'])) #  clustering_ranges 
                pv24.set_text(str(val[23]['clustering_distances'])) #  clustering_distances 
                pv25.set_text(str(val[24]['remove_ground'])) # remove_ground
                pv26.set_text(str(val[25]['use_diffnormals'])) # use_diffnormals
                pv27.set_text(str(val[26]['publish_filtered'])) # publish_filtered

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param10, param9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param11, param10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param12, param11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param13, param12, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param14, param13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param15, param14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param16, param15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param17, param16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param18, param17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param19, param18, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param20, param19, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param21, param20, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param22, param21, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param23, param22, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param24, param23, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param25, param24, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param26, param25, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param27, param26, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv10, pv9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv11, pv10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv12, pv11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv13, pv12, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv14, pv13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv15, pv14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv16, pv15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv17, pv16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv18, pv17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv19, pv18, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv20, pv19, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv21, pv20, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv22, pv21, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv23, pv22, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv24, pv23, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv25, pv24, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv26, pv25, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv27, pv26, Gtk.PositionType.BOTTOM, 1, 1)

                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9,pv10,pv11,pv12,pv13,pv14,pv15,pv16,pv17,pv18,pv19,pv20,pv21,pv22,pv23,pv24,pv25,pv26,pv27]
        elif idx1 == 1: # Follower
            if idx2 == 0: # twist filter /config/twist_filter
                param1 = Gtk.Label('lateral_accel_limit (0.0 ~ 5.0)')
                param2 = Gtk.Label('lowpass_gain_linear_x (0.0 ~ 1.0)')
                param3 = Gtk.Label('lowpass_gain_angular_z (0.0 ~ 1.0)')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()

                val = getYamlIndex(config_, 'conf_twist_filter')

                #default values
                pv1.set_text(str(val[0]['lateral_accel_limit']))
                pv2.set_text(str(val[1]['lowpass_gain_linear_x']))
                pv3.set_text(str(val[2]['lowpass_gain_angular_z']))
                
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3]
            elif idx2 == 1: # pure pursuit /config/waypoint_follwer
                param1 = Gtk.Label('Waypoint : 0, Dialog : 1')
                param2 = Gtk.Label('velocity (0.0 ~ 60.0)')
                param3 = Gtk.Label('lookahead distance (0.0 ~ 30.0)')
                param4 = Gtk.Label('lookahead ratio (0.0 ~ 2.0)')
                param5 = Gtk.Label('minimum lookahead distance (0.0 ~ 20.0)')
                param6 = Gtk.Label('displacement threshold (0.0 ~ 1.0)')
                param7 = Gtk.Label('relative angle threshold (0.0 ~ 90.0)')
                param8 = Gtk.Label('linear interpolation')
                param9 = Gtk.Label('publishs topic for steering robot')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()

                lval = getYamlIndex(launch_, 'pure_pursuit')
                cval = getYamlIndex(config_, 'conf_waypoint_follower')

                #default values
                pv1.set_text(str(cval[0]['param_flag']))
                pv2.set_text(str(cval[1]['velocity']))
                pv3.set_text(str(cval[2]['lookahead_distance']))
                pv4.set_text(str(cval[3]['lookahead_ratio']))
                pv5.set_text(str(cval[4]['minimum_lookahead_distance']))
                pv6.set_text(str(cval[5]['displacement_threshold']))
                pv7.set_text(str(cval[6]['relative_angle_threshold']))
                pv8.set_text(str(lval[0]['linear_interpolation']))
                pv9.set_text(str(lval[1]['publishs_topic_for_steering_robot']))

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9]

            elif idx2 == 2: # mpc
                param1 = Gtk.Label('show_debug_info')
                param2 = Gtk.Label('publish_debug_values')
                param3 = Gtk.Label('vehicle_model_type (kinematics, kinematics no delaym dynamics)')
                param4 = Gtk.Label('qp_solver_type (unconstraint, unconstraint_fast)')
                param5 = Gtk.Label('ctrl_period[s] (0.0 ~ 0.1)')
                param6 = Gtk.Label('admisible_position_error (0.0 ~ 100.0)')
                param7 = Gtk.Label('admisible_yaw_error_deg (0.0 ~ 360.0)')
                param8 = Gtk.Label('mpc_prediction_horizon (1.0 ~ 300.0)')
                param9 = Gtk.Label('mpc_prediction_sampling_time(0.01 ~ 1.0)')
                param10 = Gtk.Label('mpc_weight_lat_error (0.0 ~ 10.0)')
                param11 = Gtk.Label('mpc_weight_terminal_lat_error (0.0 ~ 100)')
                param12 = Gtk.Label('mpc_weight_heading_error (0.0 ~ 10.0)')
                param13 = Gtk.Label('mpc_weight_heading_error_squared_vel_coeff (0.0 ~ 10.0)')
                param14 = Gtk.Label('mpc_weight_terminal_heading_error (0.0 ~ 100.0)')
                param15 = Gtk.Label('mpc_weight_lat_jerk (0.0 ~ 10.0)')
                param16 = Gtk.Label('mpc_weight_steering_input (0.0 ~ 10.0)')
                param17 = Gtk.Label('mpc_weight_steering_input_squared_vel_coeff (0.0 ~ 10.0)')
                param18 = Gtk.Label('mpc_zero_ff_steer_deg (0.0 ~ 10.0')
                param19 = Gtk.Label('enable_path_smoothing')
                param20 = Gtk.Label('path_smoothing_times (1 ~ 3)')
                param21 = Gtk.Label('path_filter_moving_ave_num (1 ~ 50)')
                param22 = Gtk.Label('curvature_smoothing_num (1 ~ 50)')
                param23 = Gtk.Label('steering_lpf_cutoff_hz (0.1 ~ 10.0)')
                param24 = Gtk.Label('vehicle_model_steer_tau (0.001 ~ 5.0)')
                param25 = Gtk.Label('vehicle_model_wheelbase (0.1 ~ 10.0)')
                param26 = Gtk.Label('steer_lim_deg (10.0 ~ 45.0)')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()
                pv10 = Gtk.Entry()
                pv11 = Gtk.Entry()
                pv12 = Gtk.Entry()
                pv13 = Gtk.Entry()
                pv14 = Gtk.Entry()
                pv15 = Gtk.Entry()
                pv16 = Gtk.Entry()
                pv17 = Gtk.Entry()
                pv18 = Gtk.Entry()
                pv19 = Gtk.Entry()
                pv20 = Gtk.Entry()
                pv21 = Gtk.Entry()
                pv22 = Gtk.Entry()
                pv23 = Gtk.Entry()
                pv24 = Gtk.Entry()
                pv25 = Gtk.Entry()
                pv26 = Gtk.Entry()

                val = getYamlIndex(launch_, 'mpc')

                #default values
                pv1.set_text(str(val[0]['show_debug_info'])) # show_debug_info
                pv2.set_text(str(val[1]['publish_debug_values'])) # publish_debug_values
                pv3.set_text(str(val[2]['vehicle_model_type'])) # vehicle_model_type
                pv4.set_text(str(val[3]['qp_solver_type'])) # qp_solver_type
                pv5.set_text(str(val[4]['ctrl_period'])) # ctrl_period
                pv6.set_text(str(val[5]['admisible_position_error'])) # admisible_position_error
                pv7.set_text(str(val[6]['admisible_yaw_error_deg'])) # admisible_yaw_error_deg
                pv8.set_text(str(val[7]['mpc_prediction_horizon'])) # mpc_prediction_horizon
                pv9.set_text(str(val[8]['mpc_prediction_sampling_time'])) # mpc_prediction_sampling_time
                pv10.set_text(str(val[9]['mpc_weight_lat_error'])) # mpc_weight_lat_error
                pv11.set_text(str(val[10]['mpc_weight_terminal_lat_error'])) # mpc_weight_terminal_lat_error
                pv12.set_text(str(val[11]['mpc_weight_heading_error'])) # mpc_weight_heading_error
                pv13.set_text(str(val[12]['mpc_weight_heading_error_squared_vel_coeff'])) #  mpc_weight_heading_error_squared_vel_coeff
                pv14.set_text(str(val[13]['mpc_weight_terminal_heading_error'])) # mpc_weight_terminal_heading_error
                pv15.set_text(str(val[14]['mpc_weight_lat_jerk'])) # mpc_weight_lat_jerk
                pv16.set_text(str(val[15]['mpc_weight_steering_input'])) # mpc_weight_steering_input 
                pv17.set_text(str(val[16]['mpc_weight_steering_input_squared_vel_coeff'])) # mpc_weight_steering_input_squared_vel_coeff 
                pv18.set_text(str(val[17]['mpc_zero_ff_steer_deg'])) # mpc_zero_ff_steer_deg
                pv19.set_text(str(val[18]['enable_path_smoothing'])) # enable_path_smoothing
                pv20.set_text(str(val[19]['path_smoothing_times'])) # path_smoothing_times 
                pv21.set_text(str(val[20]['path_filter_moving_ave_num'])) # path_filter_moving_ave_num 
                pv22.set_text(str(val[21]['curvature_smoothing_num'])) #curvature_smoothing_num 
                pv23.set_text(str(val[22]['steering_lpf_cutoff_hz'])) #  steering_lpf_cutoff_hz 
                pv24.set_text(str(val[23]['vehicle_model_steer_tau'])) #  vehicle_model_steer_tau 
                pv25.set_text(str(val[24]['vehicle_model_wheelbase'])) # vehicle_model_wheelbase
                pv26.set_text(str(val[25]['steer_lim_deg'])) # steer_lim_deg

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param10, param9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param11, param10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param12, param11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param13, param12, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param14, param13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param15, param14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param16, param15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param17, param16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param18, param17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param19, param18, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param20, param19, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param21, param20, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param22, param21, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param23, param22, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param24, param23, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param25, param24, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param26, param25, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv10, pv9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv11, pv10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv12, pv11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv13, pv12, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv14, pv13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv15, pv14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv16, pv15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv17, pv16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv18, pv17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv19, pv18, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv20, pv19, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv21, pv20, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv22, pv21, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv23, pv22, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv24, pv23, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv25, pv24, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv26, pv25, Gtk.PositionType.BOTTOM, 1, 1)

                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9,pv10,pv11,pv12,pv13,pv14,pv15,pv16,pv17,pv18,pv19,pv20,pv21,pv22,pv23,pv24,pv25,pv26]
#            elif idx2 == 3: # hybride stenly
        elif idx1 == 2: # Localizer
            if idx2 == 0: # ndt matching
                # /config/ndt
                param1 = Gtk.Label('init_pos_gnss (Use GNSS : 1, No : 0)')
                param2 = Gtk.Label('use_predict_pose (on : 1, off : 0)')
                param3 = Gtk.Label('error_threshold (0.0 ~ 10.0)')
                param4 = Gtk.Label('resolution (0.0 ~ 10.0)')
                param5 = Gtk.Label('step_size (0.0 ~ 1.0)')
                param6 = Gtk.Label('trans_epsilon (0.0 ~ 0.1)')
                param7 = Gtk.Label('max_iterations (1 ~ 300)')

                # launch param
                param8 = Gtk.Label('method_type (pcl_generic : 0, pcl_anh : 1, pcl_anh_gpu : 2, pcl_openmp : 3)')
                param9 = Gtk.Label('use_odom')
                param10 = Gtk.Label('use_imu')
                param11 = Gtk.Label('imu_upside_down')
                param12 = Gtk.Label('imu_topic')
                param13 = Gtk.Label('get_height')
                param14 = Gtk.Label('output_log_data')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()
                pv10 = Gtk.Entry()
                pv11 = Gtk.Entry()
                pv12 = Gtk.Entry()
                pv13 = Gtk.Entry()
                pv14 = Gtk.Entry()


                lval = getYamlIndex(launch_, 'ndt')
                cval = getYamlIndex(config_, 'conf_ndt')

                #default values
                pv1.set_text(str(cval[0]['init_pos_gnss'])) # init_pos_gnss
                pv2.set_text(str(cval[1]['use_predict_pose'])) # use_predict_pose
                pv3.set_text(str(cval[2]['error_threshold'])) # error_threshold
                pv4.set_text(str(cval[3]['resolution'])) # resolution
                pv5.set_text(str(cval[4]['step_size'])) # step_size
                pv6.set_text(str(cval[5]['trans_epsilon'])) # trans_epsilon
                pv7.set_text(str(cval[6]['max_iterations'])) # max_iterations

                pv8.set_text(str(lval[0]['method_type'])) # method_type
                pv9.set_text(str(lval[1]['use_odom'])) # use_odom
                pv10.set_text(str(lval[2]['use_imu'])) # use_imu
                pv11.set_text(str(lval[3]['imu_upside_down'])) # imu_upside_down
                pv12.set_text(str(lval[4]['imu_topic'])) # imu_topic
                pv13.set_text(str(lval[5]['get_height'])) # get_height
                pv14.set_text(str(lval[6]['output_log_data'])) # output_log_data

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param10, param9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param11, param10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param12, param11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param13, param12, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param14, param13, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv10, pv9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv11, pv10, Gtk.PositionType.BOTTOM, 1,  1)
                in_grid.attach_next_to(pv12, pv11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv13, pv12, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv14, pv13, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9,pv10,pv11,pv12,pv13,pv14]
#            elif idx2 == 1: # ekf localizer
        elif idx1 == 3: # Decision Maker
            if idx2 == 0: # decision maker
                # /config/decision_maker
                param1 = Gtk.Label('auto_mission_reload')
                param2 = Gtk.Label('auto_engage')
                param3 = Gtk.Label('auto_mission_change')
                param4 = Gtk.Label('use_fms')
                param5 = Gtk.Label('disuse_vector_map')
                param6 = Gtk.Label('num_of_steer_behind (0 ~ 50)')
                param7 = Gtk.Label('goal_threshold_dist (0.1 ~ 5.0)')
                param8 = Gtk.Label('goal_threshold_vel (0.0 ~ 0.1)')
                param9 = Gtk.Label('change_threshold_dist (0.0 ~ 10.0)')
                param10 = Gtk.Label('change_threshold_angle (0.0 ~ 90.0)')
                param11 = Gtk.Label('stopped_vel (0.0 ~ 5.0)')
                # launch param
                param12 = Gtk.Label('points_topic')
                param13 = Gtk.Label('baselink_tf')
                
                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()
                pv10 = Gtk.Entry()
                pv11 = Gtk.Entry()
                pv12 = Gtk.Entry()
                pv13 = Gtk.Entry()

                lval = getYamlIndex(launch_, 'decision_maker')
                cval = getYamlIndex(config_, 'conf_decision_maker')

                #default values
                pv1.set_text(str(cval[0]['auto_mission_reload'])) # auto_mission_reload
                pv2.set_text(str(cval[1]['auto_engage'])) # auto_engage
                pv3.set_text(str(cval[2]['auto_mission_change'])) # auto_mission_change
                pv4.set_text(str(cval[3]['use_fms'])) # use_fms
                pv5.set_text(str(cval[4]['disuse_vector_map'])) # disuse_vector_map
                pv6.set_text(str(cval[5]['num_of_steer_behind'])) # num_of_steer_behind
                pv7.set_text(str(cval[8]['goal_threshold_dist'])) # goal_threshold_dist
                pv8.set_text(str(cval[9]['goal_threshold_vel'])) # goal_threshold_vel
                pv9.set_text(str(cval[6]['change_threshold_dist'])) # change_threshold_dist
                pv10.set_text(str(cval[7]['change_threshold_angle'])) # change_threshold_angle
                pv11.set_text(str(cval[10]['stopped_vel'])) # stopped_vel
                pv12.set_text(str(lval[0]['points_topic'])) # points_topic
                pv13.set_text(str(lval[1]['baselink_tf'])) # baselink_tf

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param10, param9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param11, param10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param12, param11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param13, param12, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv10, pv9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv11, pv10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv12, pv11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv13, pv12, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9,pv10,pv11,pv12,pv13]                
#        elif idx1 == 4: # LaneChange Manager
#            if idx2 == 0: # lanechange manager
        elif idx1 == 5: # Local Planner
            if idx2 == 0: # op commom params
                # launch param
                param1 = Gtk.Label('horizonDistance (1.0 ~ 250.0')
                param2 = Gtk.Label('maxLocalPlanDistance (0.5 ~ 150.0)')
                param3 = Gtk.Label('pathDensity (0.01 ~ 5.0)')
                param4 = Gtk.Label('rollOutDensity (0.01 ~ 5.0)')
                param5 = Gtk.Label('rollOutsNumber (0 ~20)')
                param6 = Gtk.Label('maxVelocity (-1.0 ~ 20.0)')
                param7 = Gtk.Label('maxAcceleration (0.01 ~ 25.0)')
                param8 = Gtk.Label('maxDeceleration (-25.0 ~ -0.01)')
                param9 = Gtk.Label('enableFollowing')
                param10 = Gtk.Label('enableSwerving')
                param11 = Gtk.Label('minFollowingDistance (0.5 ~ 100.0)')
                param12 = Gtk.Label('minDistanceToAvoid (0.1 ~ 80.0)')
                param13 = Gtk.Label('maxDistanceToAvoid (0.05 ~ 60.0)')
                param14 = Gtk.Label('enableStopSignBehavior')
                param15 = Gtk.Label('enableTrafficLightBehavior')
                param16 = Gtk.Label('enableLaneChange')
                param17 = Gtk.Label('horizontalSafetyDistance (0.0 ~ 10.0)')
                param18 = Gtk.Label('verticalSafetyDistance (0.0 ~ 25.0)')
                param19 = Gtk.Label('velocitySource (Odometry : 0, Autoware : 1, Car Info : 2)')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()
                pv10 = Gtk.Entry()
                pv11 = Gtk.Entry()
                pv12 = Gtk.Entry()
                pv13 = Gtk.Entry()
                pv14 = Gtk.Entry()
                pv15 = Gtk.Entry()
                pv16 = Gtk.Entry()
                pv17 = Gtk.Entry()
                pv18 = Gtk.Entry()
                pv19 = Gtk.Entry()

                val = getYamlIndex(launch_, 'op_common_params')

                #default values
                pv1.set_text(str(val[0]['horizonDistance'])) # horizonDistance
                pv2.set_text(str(val[1]['maxLocalPlanDistance'])) # maxLocalPlanDistance
                pv3.set_text(str(val[2]['pathDensity'])) # pathDensity
                pv4.set_text(str(val[3]['rollOutDensity'])) # rollOutDensity
                pv5.set_text(str(val[4]['rollOutsNumber'])) # rollOutsNumber
                pv6.set_text(str(val[5]['maxVelocity'])) # maxVelocity
                pv7.set_text(str(val[6]['maxAcceleration'])) # maxAcceleration
                pv8.set_text(str(val[7]['maxDeceleration'])) # maxDeceleration
                pv9.set_text(str(val[8]['enableFollowing'])) # enableFollowing
                pv10.set_text(str(val[9]['enableSwerving'])) # enableSwerving
                pv11.set_text(str(val[10]['minFollowingDistance'])) # minFollowingDistance
                pv12.set_text(str(val[11]['minDistanceToAvoid'])) # minDistanceToAvoid
                pv13.set_text(str(val[12]['maxDistanceToAvoid'])) # maxDistanceToAvoid
                pv14.set_text(str(val[13]['enableStopSignBehavior'])) # enableStopSignBehavior
                pv15.set_text(str(val[14]['enableTrafficLightBehavior'])) # enableTrafficLightBehavior
                pv16.set_text(str(val[15]['enableLaneChange'])) # enableLaneChange 
                pv17.set_text(str(val[16]['horizontalSafetyDistance'])) # horizontalSafetyDistance 
                pv18.set_text(str(val[17]['verticalSafetyDistance'])) # verticalSafetyDistance
                pv19.set_text(str(val[18]['velocitySource'])) # velocitySource

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param10, param9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param11, param10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param12, param11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param13, param12, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param14, param13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param15, param14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param16, param15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param17, param16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param18, param17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param19, param18, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv10, pv9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv11, pv10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv12, pv11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv13, pv12, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv14, pv13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv15, pv14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv16, pv15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv17, pv16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv18, pv17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv19, pv18, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9,pv10,pv11,pv12,pv13,pv14,pv15,pv16,pv17,pv18,pv19]
            elif idx2 == 1: # op trajectory generator
                param1 = Gtk.Label('samplingTipMargin (0.1 ~ 10.0)')
                param2 = Gtk.Label('samplingOutMargin (0.2 ~ 40.0)')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()

                val = getYamlIndex(launch_, 'op_trajectory_generator')

                pv1.set_text(str(val[0]['samplingTipMargin'])) # samplingTipMargin
                pv2.set_text(str(val[1]['samplingOutMargin'])) # samplingOutMargin

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2]
            elif idx2 == 2: # op motion predictor
                # launch params
                param1 = Gtk.Label('enableCurbObstacles')
                param2 = Gtk.Label('enableGenrateBranches')
                param3 = Gtk.Label('max_distance_to_lane (0.0 ~ 10.0)')
                param4 = Gtk.Label('prediction_distance (1.0 ~ 75.0)')
                param5 = Gtk.Label('enableStepByStepSignal')
                param6 = Gtk.Label('enableParticleFilterPrediction')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()

                val = getYamlIndex(launch_, 'op_motion_predictor')

                #default values
                pv1.set_text(str(val[0]['enableCurbObstacles'])) # enableCurbObstacles
                pv2.set_text(str(val[1]['enableGenrateBranches'])) # enableGenrateBranches
                pv3.set_text(str(val[2]['max_distance_to_lane'])) # max_distance_to_lane
                pv4.set_text(str(val[3]['prediction_distance'])) # prediction_distance
                pv5.set_text(str(val[4]['enableStepByStepSignal'])) # enableStepByStepSignal
                pv6.set_text(str(val[5]['enableParticleFilterPrediction'])) # enableParticleFilterPrediction

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv3,pv4,pv5,pv6]
            elif idx2 == 3: # op trajectory evaluator
                param1 = Gtk.Label('enablePrediction')
                pv1 = Gtk.Entry()
                val = getYamlIndex(launch_, 'op_trajectory_evaluator')
                pv1.set_text(str(val[0]['enablePrediction'])) # enablePrediction
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                param_list = [pv1]
#            elif idx2 == 4: # op behavior selector
        elif idx1 == 6: # Vehicle Setting
            if idx2 == 0: # vel pose connect
                # launch params
                param1 = Gtk.Label('topic_pose_stamped')
                param2 = Gtk.Label('topic_twist_stamped')
                param3 = Gtk.Label('sim_mode')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()

                val = getYamlIndex(launch_, 'vel_pose_connect')

                pv1.set_text(str(val[0]['topic_pose_stamped'])) # topic_pose_stamped
                pv2.set_text(str(val[1]['topic_twist_stamped'])) # topic_twist_stamped
                pv3.set_text(str(val[2]['sim_mode'])) # sim_mode

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3]
            elif idx2 == 1: # baselink to localizer
                # launch params
                param1 = Gtk.Label('x')
                param2 = Gtk.Label('y')
                param3 = Gtk.Label('z')
                param4 = Gtk.Label('yaw')
                param5 = Gtk.Label('pitch')
                param6 = Gtk.Label('roll')
                param7 = Gtk.Label('frame_id')
                param8 = Gtk.Label('child_frame_id')
                param9 = Gtk.Label('period_in_ms')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()

                val = getYamlIndex(launch_, 'baselink_to_localizer')

                #default values
                pv1.set_text(str(val[0]['x'])) # x
                pv2.set_text(str(val[1]['y'])) # y
                pv3.set_text(str(val[2]['z'])) # z
                pv4.set_text(str(val[3]['yaw'])) # yaw
                pv5.set_text(str(val[4]['pitch'])) # pitch
                pv6.set_text(str(val[5]['roll'])) # roll
                pv7.set_text(str(val[6]['frame_id'])) # frame_id
                pv8.set_text(str(val[7]['child_frame_id'])) # child_frame_id
                pv9.set_text(str(val[8]['period_in_ms'])) # period_in_ms

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9]
        # elif idx1 == 7: # quick start
        #     if idx2 == 0: # all quick start
            
        exec_button.connect("clicked", self.onClickedExcute, idx1, idx2, param_list, setparam_win)
        setparam_win.show_all()
        # 체크 눌리면 파라미터 세팅하는 창 만들기 고민 하다가 집갔음.
        # 함수에서 명령어를 실행할건지 아니면 체크박스가 눌리면 불리는 콜백함수에서 실행할건지 고민 <- 리턴값이 애매해서 힘들듯
        #exec_button.connect("clicked", self.on_click_me_clicked)
        
    def getParamSettingWin2(self, idx1, idx2):
        setparam_win = Gtk.Window()
        setparam_win.set_title("set params")
        setparam_win.set_default_size(300, 400)
        setparam_win.set_border_width(20)
        grid = Gtk.Grid()
        grid.set_column_homogeneous(True)
        grid.set_row_homogeneous(True)
        
        #debug_label = Gtk.Label(str(idx1) + str(idx2))
        setparam_win.add(grid)

        set_param_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        grid.attach(set_param_box, 0, 0, 5, 10)

        in_grid = Gtk.Grid()
        in_grid.set_column_homogeneous(True)
        in_grid.set_row_homogeneous(True)
        exec_button = Gtk.Button.new_with_label("Execute")
        
        launch_ = default_yaml.get('launch_param', [])
        config_ = default_yaml.get('config', [])

        grid.attach_next_to(
            exec_button, set_param_box, Gtk.PositionType.BOTTOM, 1, 1
        )
        set_param_box.pack_start(in_grid, True, True, 0)
        param_list = []
        if idx1 == 0: # Map
            if idx2 == 0: # point cloud
                param1 = Gtk.Label('PCD MAP path')
                pv1 = Gtk.Entry()
                val = getYamlIndex(launch_, 'point_cloud_loader')
                #default values
                pv1.set_text(str(val[0]['path']))
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                param_list = [pv1]
            elif idx2 == 1: # vector map
                param1 = Gtk.Label('Vector MAP path ( path1 path2 path3 ... )')
                pv1 = Gtk.Entry()
                val = getYamlIndex(launch_, 'vector_map_loader')
                #default values
                pv1.set_text(str(val[0]['path']))
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                param_list = [pv1]
            elif idx2 == 2: # point vector tf
                param1 = Gtk.Label('tf launch path')
                pv1 = Gtk.Entry()
                val = getYamlIndex(launch_, 'point_vector_tf')
                #default values
                pv1.set_text(str(val[0]['path']))
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                param_list = [pv1]
        elif idx1 == 1: # Sensing
            if idx2 == 0: # ray_ground_filter
                param1 = Gtk.Label('input_point_topic')
                param2 = Gtk.Label('sensor_height (-5.0 ~ 5.0)')
                param3 = Gtk.Label('clipping_height (-5.0 ~ 5.0)')
                param4 = Gtk.Label('min_point_distance (0.0 ~ 5.0)')
                param5 = Gtk.Label('radial_divider_angle (0.01 ~ 5.0)')
                param6 = Gtk.Label('concentric_divider_distance (0.01 ~ 1.0)')
                param7 = Gtk.Label('local_max_slope (1 ~ 25)')
                param8 = Gtk.Label('general_max_slope (1 ~ 25)')
                param9 = Gtk.Label('min_height_threshold (0.01 ~ 0.5)')
                param10 = Gtk.Label('reclass_distance_threshold (0.01 ~ 1.0)')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()
                pv5 = Gtk.Entry()
                pv6 = Gtk.Entry()
                pv7 = Gtk.Entry()
                pv8 = Gtk.Entry()
                pv9 = Gtk.Entry()
                pv10 = Gtk.Entry()

                val = getYamlIndex(config_, 'conf_ray_ground_filter')

                #default values
                pv1.set_text('/points_raw')
                pv2.set_text(str(val[0]['sensor_height'])) 
                pv3.set_text(str(val[1]['clipping_height'])) 
                pv4.set_text(str(val[2]['min_point_distance'])) 
                pv5.set_text(str(val[3]['radial_divider_angle']))
                pv6.set_text(str(val[4]['concentric_divider_distance'])) 
                pv7.set_text(str(val[5]['local_max_slope'])) 
                pv8.set_text(str(val[6]['general_max_slope'])) 
                pv9.set_text(str(val[7]['min_height_threshold'])) 
                pv10.set_text(str(val[8]['reclass_distance_threshold']))

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param10, param9, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv5, pv4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv6, pv5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv7, pv6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv8, pv7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv9, pv8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv10, pv9, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv4,pv5,pv6,pv7,pv8,pv9,pv10]
            elif idx2 == 1: # cloud_transformer
                param1 = Gtk.Label('input_point_topic')
                param2 = Gtk.Label('target_frame')
                param3 = Gtk.Label('output_point_topic')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()

                val = getYamlIndex(launch_, 'cloud_transformer')

                pv1.set_text(str(val[0]['input_point_topic'])) 
                pv2.set_text(str(val[1]['target_frame'])) 
                pv3.set_text(str(val[2]['output_point_topic'])) 

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3]
            #elif idx2 == 2: # sensor3
            #elif idx2 == 3: # sensor4
        elif idx1 == 2: # Point Downsampler
            if idx2 == 0: # voxel grid filter
                # launch params
                param1 = Gtk.Label('node_name')
                param2 = Gtk.Label('points_topic')
                # /config/voxel_grid_filter
                param3 = Gtk.Label('voxel_leaf_size (0.0 ~ 10.0)')
                param4 = Gtk.Label('measurement_range (0 ~ 200)')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                pv4 = Gtk.Entry()

                lval = getYamlIndex(launch_, 'voxel_grid_filter')
                cval = getYamlIndex(config_, 'conf_voxel_grid_filter')

                pv1.set_text(str(lval[0]['node_name'])) # node_name
                pv2.set_text(str(lval[1]['points_topic'])) # points_topic
                pv3.set_text(str(cval[0]['voxel_leaf_size'])) # voxel_leaf_size
                pv4.set_text(str(cval[1]['measurement_range'])) # measurement_range

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv4, pv3, Gtk.PositionType.BOTTOM, 1, 1)
                param_list = [pv1,pv2,pv3,pv4]
        # elif idx1 == 3: # quick start
        #     if idx2 == 0: # map quick start
        #     elif idx2 == 1: # sensing quick start

        exec_button.connect("clicked", self.onClickedExcute2, idx1, idx2, param_list, setparam_win)
        setparam_win.show_all()
    
    def onExcuteThread(self, cmd):
        exec_thread = threading.Thread(target=self.excuteCore, args=(cmd,))
        exec_thread.daemon = True
        exec_thread.start()

    def excuteCore(self, cmd):
        subprocess.check_call([cmd], shell=True)

    def onClickedExcute(self, widget, idx1, idx2, plist, window):
        if idx1 == 0: # Detection
            if idx2 == 0: # lidar detector
                run_cmd = 'roslaunch lidar_detect qt_detect_launch.launch'
                node_sequence_list.append('/obb_generator /qt_detect_node')
            elif idx2 == 1: # camera detector
                run_cmd = 'None'
                #run_cmd = ('roslaunch vision_darknet_detect vision_yolo3_detect.launch'
                #            + ' score_threshold:='          + plist[].get_text()
                #            + ' nms_threshold:'             + plist[].get_text()
                #            + ' image_src:'                 + plist[].get_text()
                #            + ' network_definition_file:='  + plist[].get_text() 
                #            + ' pretrained_model_file:='    + plist[].get_text()
                #            + ' names_file:='               + plist[].get_text()
                #            + ' gpu_device_id:'             + plist[].get_text()
                #            + ' camera_id:='                + plist[].get_text())
                node_sequence_list.append('/vision_darknet_detect /yolo3_rects')
            elif idx2 == 2: # lidar kf contour track
                run_cmd = ('roslaunch lidar_kf_contour_track lidar_kf_contour_track.launch'
                          + ' tracking_type:='              + plist[0].get_text() 
                          + ' min_object_size:='            + plist[1].get_text() 
                          + ' max_object_size:='            + plist[2].get_text() 
                          + ' vector_map_filter_distance:=' + plist[3].get_text() 
                          + ' enableLogging:='              + plist[4].get_text() 
                          + ' polygon_quarters:='           + plist[5].get_text() 
                          + ' polygon_resolution:='         + plist[6].get_text() 
                          + ' max_association_distance:='   + plist[7].get_text() 
                          + ' max_association_size_diff:='  + plist[8].get_text() 
                          + ' max_remeber_time:='           + plist[9].get_text() 
                          + ' trust_counter:='              + plist[10].get_text() 
                          + ' enableSimulationMode:='       + plist[11].get_text() 
                          + ' enableStepByStepMode:='       + plist[12].get_text())
                node_sequence_list.append('/lidar_kf_contour_track')
            elif idx2 == 3: # lidar camera fusion
                run_cmd = 'None'
                node_sequence_list.append('/detection/fusion_tools/range_fusion_visualization_01 /range_vision_fusion_01')
            elif idx2 == 4: # lidar_euclidean_cluster_detect
                run_cmd = ('roslaunch lidar_euclidean_cluster_detect lidar_euclidean_cluster_detect.launch'
                            + ' use_gpu:='                  + plist[0].get_text()
                            + ' output_frame:='             + plist[1].get_text()
                            + ' pose_estimation:='          + plist[2].get_text()
                            + ' downsample_cloud:='         + plist[3].get_text()
                            + ' points_node:='              + plist[4].get_text()
                            + ' leaf_size:='                + plist[5].get_text()
                            + ' cluster_size_min:='         + plist[6].get_text()
                            + ' cluster_size_max:='         + plist[7].get_text()
                            + ' clustering_distance:='      + plist[8].get_text()
                            + ' clip_min_height:='          + plist[9].get_text()
                            + ' clip_max_height:='          + plist[10].get_text()
                            + ' use_vector_map:='           + plist[11].get_text()
                            + ' vectormap_frame:='          + plist[12].get_text()
                            + ' wayarea_gridmap_topic:='    + plist[13].get_text()
                            + ' wayarea_gridmap_layer:='    + plist[14].get_text()
                            + ' wayarea_no_road_value:='    + plist[15].get_text()
                            + ' remove_points_upto:='       + plist[16].get_text()
                            + ' keep_lanes:='               + plist[17].get_text()
                            + ' keep_lane_left_distance:='  + plist[18].get_text()
                            + ' keep_lane_right_distance:=' + plist[19].get_text()
                            + ' cluster_merge_threshold:='  + plist[20].get_text()
                            + ' use_multiple_thres:='       + plist[21].get_text()
                            + ' clustering_ranges:='        + plist[22].get_text()
                            + ' clustering_distances:='     + plist[23].get_text()
                            + ' remove_ground:='            + plist[24].get_text()
                            + ' use_diffnormals:='          + plist[25].get_text()
                            + ' publish_filtered:='         + plist[26].get_text())
                node_sequence_list.append('/detection/lidar_detector/cluster_detect_visualization_01 /lidar_euclidean_cluster_detect')
        elif idx1 == 1: # Follower
            if idx2 == 0: # twist filter /config/twist_filter
                data = ConfigTwistFilter()
                data.lateral_accel_limit    = float(plist[0].get_text())
                data.lowpass_gain_linear_x  = float(plist[1].get_text())
                data.lowpass_gain_angular_z = float(plist[2].get_text())
                self.config_pub.onConfigTwistFilter(data)
                run_cmd = 'roslaunch waypoint_follower twist_filter.launch'
                node_sequence_list.append('/twist_filter /twist_gate')
            elif idx2 == 1: # pure pursuit /config/waypoint_follower
                data = ConfigWaypointFollower()
                data.param_flag                 = int(plist[0].get_text())
                data.velocity                   = float(plist[1].get_text())
                data.lookahead_distance         = float(plist[2].get_text())
                data.lookahead_ratio            = float(plist[3].get_text())
                data.minimum_lookahead_distance = float(plist[4].get_text())
                data.displacement_threshold     = float(plist[5].get_text())
                data.relative_angle_threshold   = float(plist[6].get_text())
                self.config_pub.onConfigWaypointFollower(data)
                run_cmd = ('roslaunch waypoint_follower pure_pursuit.launch'
                          + ' s_linear_interpolation:=' + plist[7].get_text() 
                          + ' publishes_for_steering_robot:=' + plist[8].get_text())
                node_sequence_list.append('/pure_pursuit')
            elif idx2 == 2: # mpc
                run_cmd = ('roslaunch waypoint_follower mpc_follower.launch'
                          + ' show_debug_info:='                                + plist[0].get_text() 
                          + ' publish_debug_values:='                           + plist[1].get_text() 
                          + ' vehicle_model_type:='                             + plist[2].get_text() 
                          + ' qp_solver_type:='                                 + plist[3].get_text() 
                          + ' admisible_position_error:='                       + plist[4].get_text() 
                          + ' admisible_yaw_error_deg:='                        + plist[5].get_text() 
                          + ' mpc_prediction_horizon:='                         + plist[6].get_text() 
                          + ' mpc_prediction_sampling_time:='                   + plist[7].get_text() 
                          + ' mpc_weight_lat_error:='                           + plist[8].get_text() 
                          + ' mpc_weight_terminal_lat_error:='                  + plist[9].get_text() 
                          + ' mpc_weight_heading_error:='                       + plist[10].get_text() 
                          + ' mpc_weight_heading_error_squared_vel_coeff:='     + plist[11].get_text() 
                          + ' mpc_weight_terminal_heading_error:='              + plist[12].get_text() 
                          + ' mpc_weight_lat_jerk:='                            + plist[13].get_text() 
                          + ' mpc_weight_steering_input:='                      + plist[14].get_text() 
                          + ' mpc_weight_steering_input_squared_vel_coeff:='    + plist[15].get_text() 
                          + ' mpc_zero_ff_steer_deg:='                          + plist[16].get_text() 
                          + ' enable_path_smoothing:='                          + plist[17].get_text() 
                          + ' path_smoothing_times:='                           + plist[18].get_text() 
                          + ' path_filter_moving_ave_num:='                     + plist[19].get_text() 
                          + ' curvature_smoothing_num:='                        + plist[20].get_text() 
                          + ' steering_lpf_cutoff_hz:='                         + plist[21].get_text() 
                          + ' vehicle_model_steer_tau:='                        + plist[22].get_text() 
                          + ' vehicle_model_wheelbase:='                        + plist[23].get_text() 
                          + ' steer_lim_deg:='                                  + plist[24].get_text())
                node_sequence_list.append('/mpc_follower /mpc_waypoints_converter')
            elif idx2 == 3: # hybride stenly
                run_cmd = 'None'
                node_sequence_list.append('/hybride_stenly')
        elif idx1 == 2: # Localizer
            if idx2 == 0: # ndt matching
                data = ConfigNDT()
                data.init_pos_gnss    = int(plist[0].get_text())
                data.use_predict_pose = int(plist[1].get_text())
                data.error_threshold  = float(plist[2].get_text())
                data.resolution       = float(plist[3].get_text())
                data.step_size        = float(plist[4].get_text())
                data.trans_epsilon    = float(plist[5].get_text())
                data.max_iterations   = int(plist[6].get_text())
                self.config_pub.onConfigNdt(data)
                run_cmd = ('roslaunch lidar_localizer ndt_matching.launch'
                + ' method_type:='      + plist[7].get_text() 
                + ' use_odom:='         + plist[8].get_text() 
                + ' use_imu:='          + plist[9].get_text() 
                + ' imu_upside_down:='  + plist[10].get_text() 
                + ' imu_topic:='        + plist[11].get_text() 
                + ' get_height:='       + plist[12].get_text() 
                + ' output_log_data:='  + plist[13].get_text())
                node_sequence_list.append('/ndt_matching')
            elif idx2 == 1: # ekf localizer
                run_cmd = 'None'
                node_sequence_list.append('/ekf_localizer')
        elif idx1 == 3: # Decision Maker
            if idx2 == 0: # decision maker
                data = ConfigDecisionMaker()
                data.auto_mission_reload    = bool(plist[0].get_text())
                data.auto_engage            = bool(plist[1].get_text())
                data.auto_mission_change    = bool(plist[2].get_text())
                data.use_fms                = bool(plist[3].get_text())
                data.disuse_vector_map      = bool(plist[4].get_text())
                data.num_of_steer_behind    = int(plist[5].get_text())
                data.change_threshold_dist  = float(plist[8].get_text())
                data.change_threshold_angle = float(plist[9].get_text())
                data.goal_threshold_dist    = float(plist[6].get_text())
                data.goal_threshold_vel     = float(plist[7].get_text())
                data.stopped_vel            = float(plist[10].get_text())
                self.config_pub.onConfigDecisionMaker(data)
                run_cmd = ('roslaunch decision_maker decision_maker.launch'
                          + ' disuse_vector_map:='      + plist[4].get_text() 
                          + ' points_topic:='           + plist[11].get_text() 
                          + ' baselink_tf:='            + plist[12].get_text() 
                          + ' use_fms:='                + plist[3].get_text() 
                          + ' auto_mission_reload:='    + plist[0].get_text() 
                          + ' auto_engage:='            + plist[1].get_text() 
                          + ' auto_mission_change:='    + plist[2].get_text())
                node_sequence_list.append('/decision_maker')
        elif idx1 == 4: # LaneChange Manager
            if idx2 == 0: # lanechange manager
                run_cmd = 'roslaunch lanechange_manager lanechange_manager.launch'
                node_sequence_list.append('/lanechange_manager')
        elif idx1 == 5: # Local Planner
            if idx2 == 0: # op commom params
                run_cmd = ('roslaunch op_local_planner op_common_params.launch'
                          +' horizonDistance:='             + plist[0].get_text() 
                          + ' maxLocalPlanDistance:='       + plist[1].get_text() 
                          + ' pathDensity:='                + plist[2].get_text() 
                          + ' rollOutDensity:='             + plist[3].get_text() 
                          + ' rollOutsNumber:='             + plist[4].get_text() 
                          + ' maxVelocity:='                + plist[5].get_text() 
                          + ' maxAcceleration:='            + plist[6].get_text() 
                          + ' maxDeceleration:='            + plist[7].get_text() 
                          + ' enableFollowing:='            + plist[8].get_text() 
                          + ' enableSwerving:='             + plist[9].get_text() 
                          + ' minFollowingDistance:='       + plist[10].get_text() 
                          + ' minDistanceToAvoid:='         + plist[11].get_text() 
                          + ' maxDistanceToAvoid:='         + plist[12].get_text() 
                          + ' enableStopSignBehavior:='     + plist[13].get_text() 
                          + ' enableTrafficLightBehavior:=' + plist[14].get_text() 
                          + ' enableLaneChange:='           + plist[15].get_text() 
                          + ' horizontalSafetyDistance:='   + plist[16].get_text() 
                          + ' verticalSafetyDistance:='     + plist[17].get_text() 
                          + ' velocitySource:='             + plist[18].get_text())
                node_sequence_list.append('/op_common_params')
            elif idx2 == 1: # op trajectory generator
                run_cmd = ('roslaunch op_local_planner op_trajectory_generator.launch'
                          + ' samplingTipMargin:=' + plist[0].get_text() 
                          + ' samplingOutMargin:=' + plist[1].get_text())
                node_sequence_list.append('/op_trajectory_generator')
            elif idx2 == 2: # op motion predictor
                run_cmd = ('roslaunch op_local_planner op_motion_predictor.launch'
                          + ' enableCurbObstacles:='            + plist[0].get_text() 
                          + ' enableGenrateBranches:='          + plist[1].get_text() 
                          + ' max_distance_to_lane:='           + plist[2].get_text() 
                          + ' prediction_distance:='            + plist[3].get_text() 
                          + ' enableStepByStepSignal:='         + plist[4].get_text() 
                          + ' enableParticleFilterPrediction:=' + plist[5].get_text())
                node_sequence_list.append('/op_motion_predictor')
            elif idx2 == 3: # op trajectory evaluator
                run_cmd = ('roslaunch op_local_planner op_trajectory_evaluator.launch'
                        + ' enablePrediction:=' + plist[0].get_text())
                node_sequence_list.append('/op_trajectory_evaluator')
            elif idx2 == 4: # op behavior selector
                run_cmd = 'roslaunch op_local_planner op_behavior_selector.launch'
                node_sequence_list.append('/op_behavior_selector')
        elif idx1 == 6: # Vehicle Setting
            if idx2 == 0: # vel pose connect
                run_cmd = ('roslaunch autoware_connector vel_pose_connect.launch'
                        + ' topic_pose_stamped:='   + plist[0].get_text() 
                        + ' topic_twist_stamped:='  + plist[1].get_text() 
                        + ' sim_mode:='             + plist[2].get_text())
                node_sequence_list.append('/pose_relay /vel_relay')
            elif idx2 == 1: # baselink to localizer
                run_cmd = ('roslaunch runtime_manager setup_tf.launch'
                          + ' x:='              + plist[0].get_text()
                          + ' y:='              + plist[1].get_text()
                          + ' z:='              + plist[2].get_text()
                          + ' yaw:='            + plist[3].get_text()
                          + ' pitch:='          + plist[4].get_text()
                          + ' roll:='           + plist[5].get_text()
                          + ' frame_id:='       + plist[6].get_text()
                          + ' child_frame_id:=' + plist[7].get_text()
                          + ' period_in_ms:='   + plist[8].get_text())
                node_sequence_list.append('/base_link_to_localizer')
        elif idx1 == 7: # quick start
            if idx2 == 0: # detection quick start
                run_cmd = 'None'
                self.doQuickStart(detection_quickstart_yaml, 1)
            elif idx2 == 1: # planning quick start
                run_cmd = 'None'
                self.doQuickStart(planning_quickstart_yaml, 1)

        if run_cmd == 'None':
            window.close()
        else:
            print('[ '+run_cmd+' ]')
            self.onExcuteThread(run_cmd)
            inst_sequence_list.append(run_cmd)
            window.close()

    def onClickedExcute2(self, widget, idx1, idx2, plist, window):
        if idx1 == 0: # Map
            if idx2 == 0: # point cloud
                run_cmd = ('rosrun map_file points_map_loader noupdate '
                          + plist[0].get_text())
                node_sequence_list.append('/points_map_loader')
            elif idx2 == 1: # vector map
                run_cmd = ('rosrun map_file vector_map_loader '
                          + plist[0].get_text())
                node_sequence_list.append('/vector_map_loader')
            elif idx2 == 2: # point vector tf
                run_cmd = ('roslaunch '
                            + plist[0].get_text())
                node_sequence_list.append('/world_to_map')
        elif idx1 == 1: # Sensing
            if idx2 == 0: # ray_ground_filter
                data = ConfigRayGroundFilter()
                data.sensor_height               = float(plist[1].get_text())
                data.clipping_height             = float(plist[2].get_text())
                data.min_point_distance          = float(plist[3].get_text())
                data.radial_divider_angle        = float(plist[4].get_text())
                data.concentric_divider_distance = float(plist[5].get_text())
                data.local_max_slope             = float(plist[6].get_text())
                data.general_max_slope           = float(plist[7].get_text())
                data.min_height_threshold        = float(plist[8].get_text())
                data.reclass_distance_threshold  = float(plist[9].get_text())
                run_cmd = ('roslaunch points_preprocessor ray_ground_filter.launch node_name:=ray_ground_filter'
                          + ' input_point_topic:='           + plist[0].get_text()
                          + ' sensor_height:='               + plist[1].get_text()
                          + ' clipping_height:='             + plist[2].get_text()
                          + ' min_point_distance:='          + plist[3].get_text()
                          + ' radial_divider_angle:='        + plist[4].get_text()
                          + ' concentric_divider_distance:=' + plist[5].get_text()
                          + ' local_max_slope:='             + plist[6].get_text()
                          + ' general_max_slope:='           + plist[7].get_text()
                          + ' min_height_threshold:='        + plist[8].get_text()
                          + ' reclass_distance_threshold:='  + plist[9].get_text())
                node_sequence_list.append('/ray_ground_filter')
            elif idx2 == 1: # cloud_transformer
                run_cmd = ('roslaunch points_preprocessor cloud_transformer.launch'
                          + ' input_point_topic:='  + plist[0].get_text()    
                          + ' target_frame:='       + plist[1].get_text()
                          + ' output_point_topic:=' + plist[2].get_text())
                node_sequence_list.append('/cloud_transformer')
            #elif idx2 == 2: # sensor3
            #elif idx2 == 3: # sensor4
        elif idx1 == 2: # Point Downsampler
            if idx2 == 0: # voxel grid filter
                data = ConfigVoxelGridFilter()
                data.voxel_leaf_size = float(plist[2].get_text())
                data.measurement_range = float(plist[3].get_text())
                self.config_pub.onConfigVoxelGridFilter(data)
                run_cmd = ('roslaunch points_downsampler points_downsample.launch'
                          + ' node_name:=' + plist[0].get_text()
                          + ' points_topic:=' + plist[1].get_text())
                node_sequence_list.append('/voxel_grid_filter')
        elif idx1 == 3: # quick start
            if idx2 == 0: # map quick start
                run_cmd = 'None'
                self.doQuickStart(map_quickstart_yaml, 2)
            elif idx2 == 1: # sensing quick start
                run_cmd = 'None'
                self.doQuickStart(sensing_quickstart_yaml, 2)
        if run_cmd == 'None':
            window.close()
        else:
            print('[ '+run_cmd+' ]')
            self.onExcuteThread(run_cmd)
            inst_sequence_list.append(run_cmd)
            window.close()

    def killNode(self, component, idx1, idx2): # set free state of seleted node
        if component == 1:
            if idx1 == 7:
                if idx2 == 0:
                    self.killQuickStart(detection_quickstart_yaml, component)
                elif idx2 == 1:
                    self.killQuickStart(planning_quickstart_yaml, component)
            elif kill_instruction[idx1][idx2] in node_sequence_list:
                os.system("rosnode kill " + kill_instruction[idx1][idx2])
                del_idx = node_sequence_list.index(kill_instruction[idx1][idx2])
                del node_sequence_list[del_idx]
                del inst_sequence_list[del_idx]
        elif component == 2:
            if idx1 == 3:
                if idx2 == 0:
                    self.killQuickStart(map_quickstart_yaml, component)
                elif idx2 == 1:
                    self.killQuickStart(sensing_quickstart_yaml, component)
            elif kill_instruction2[idx1][idx2] in node_sequence_list:
                os.system("rosnode kill " + kill_instruction2[idx1][idx2])
                del_idx = node_sequence_list.index(kill_instruction2[idx1][idx2])
                del node_sequence_list[del_idx]
                del inst_sequence_list[del_idx]

    def killQuickStart(self, yaml, component):
        quick_nodes = yaml.get('nodelist', [])

        i = 0
        if component == 1:
            for node_list in kill_instruction:
                j = 0
                for st in node_list:
                    for qn in quick_nodes:
                        if qn == st:
                            path = str(i)+':'+str(j)
                            self.node_tree[path][1] = False
                    j = j + 1
                i = i + 1
        elif component == 2:
            for node_list in kill_instruction2:
                j = 0
                for st in node_list:
                    for qn in quick_nodes:
                        if qn == st:
                            path = str(i)+':'+str(j)
                            self.node_tree2[path][1] = False
                    j = j + 1
                i = i + 1
        else:
            print("Error exception in killQuickStart!")
            exit(1)

        for node in quick_nodes:
            if node in node_sequence_list:
                os.system("rosnode kill " + node)
                del_idx = node_sequence_list.index(node)
                del node_sequence_list[del_idx]
                del inst_sequence_list[del_idx]
        
    def doQuickStart(self, yaml, component):       
        quick_list = yaml.get('routine', [])
        quick_nodes = yaml.get('nodelist', [])
        node_sequence_list.extend(quick_nodes)

        i = 0
        if component == 1:
            for node_list in kill_instruction:
                j = 0
                for st in node_list:
                    for qn in quick_nodes:
                        if qn == st:
                            path = str(i)+':'+str(j)
                            self.node_tree[path][1] = True
                    j = j + 1
                i = i + 1
        elif component == 2:
            for node_list in kill_instruction2:
                j = 0
                for st in node_list:
                    for qn in quick_nodes:
                        if qn == st:
                            path = str(i)+':'+str(j)
                            self.node_tree2[path][1] = True
                    j = j + 1
                i = i + 1
        else:
            print("Error exception in doQuickStart!")
            exit(1)

        for inst in quick_list:
            cmd = inst['instruction']
            if inst['parameters'] != None: 
                for par in  inst['parameters']:
                    cmd = cmd + ' ' + par['args']
            print('[ '+cmd+' ]')
            inst_sequence_list.append(cmd)
            self.onExcuteThread(cmd)

    def doSystemExit(self): # if system has estop state, it save all nodes info before kill all nodes
        print("\nEXIT MANAGER! Killing all nodes...")
        for i in range(len(node_sequence_list)):
            os.system("rosnode kill " + node_sequence_list[i])
        print("EXIT routine... Done")

    def doSystemEstop(self, widget = None): # if system has estop state, it save all nodes info before kill all nodes
        global estop_state
        print("\nE-STOP!! Killing all nodes...")
        for i in range(len(node_sequence_list)):
            os.system("rosnode kill " + node_sequence_list[i])
        print("E-stop routine... Done")
        estop_state = True

    def doRerunRoutine(self, widget = None): # do rerun routine, using saved all nodes info
        global estop_state
        if estop_state == False:
            print('E-STOP state is False.')
            return

        print("\nDo rerun routine")
        print("----------- rerun node list -----------")
        for i in range(len(node_sequence_list)):
            print(node_sequence_list[i])
        print("---------------------------------------")

        for i in range(len(inst_sequence_list)):
            # time.sleep(1)
            self.onExcuteThread(inst_sequence_list[i])
            # print(inst_sequence_list[i])

        estop_state = False

        # callback function for the signal emitted by the cellrenderertoggle
    def onToggled(self, widget, path):
        # the boolean value of the selected row
        current_value = self.node_tree[path][1]
        # change the boolean value of the selected row in the model
        self.node_tree[path][1] = not current_value
        # new current value!
        current_value = not current_value
        path_idx = path.split(':')
        
        if len(path_idx) == 2:
            idx1 = int(path_idx[0])
            idx2 = int(path_idx[1])
            if current_value == True:
                #print(instruction[idx1][idx2])
                self.getParamSettingWin(idx1,idx2)
            else :
                self.killNode(1, idx1, idx2)
        
        # if length of the path is 1 (that is, if we are selecting an author)
        if len(path) == 1:
            # get the iter associated with the path 
            parent_iter = self.node_tree.get_iter(path)
            # get the iter associated with its first child
            child_iter = self.node_tree.iter_children(parent_iter)
            # while there are children, change the state of their boolean value
            # to the value of the author
            while child_iter is not None:
                self.node_tree[child_iter][1] = current_value
                child_iter = self.node_tree.iter_next(child_iter)
        # if the length of the path is not 1 (that is, if we are selecting a
        # book)
        elif len(path) != 1:
            # get the first child of the parent of the book (the first book of
            # the author)
            child_iter = self.node_tree.get_iter(path)
            parent_iter = self.node_tree.iter_parent(child_iter)
            child_iter = self.node_tree.iter_children(parent_iter)
            # check if all the children are selected
            all_selected = True
            while child_iter is not None:
                if self.node_tree[child_iter][1] == False:
                    all_selected = False
                    break
                child_iter = self.node_tree.iter_next(child_iter)
            # if they do, the author as well is selected; otherwise it is not
            self.node_tree[parent_iter][1] = all_selected
    
    def onToggled2(self, widget, path):
        current_value = self.node_tree2[path][1]
        self.node_tree2[path][1] = not current_value
        current_value = not current_value
        path_idx = path.split(':')
        
        if len(path_idx) == 2:
            idx1 = int(path_idx[0])
            idx2 = int(path_idx[1])
            if current_value == True:
                # print(instruction2[idx1][idx2])
                self.getParamSettingWin2(idx1,idx2)
            else :
                self.killNode(2, idx1, idx2)

        if len(path) == 1:
            parent_iter2 = self.node_tree2.get_iter(path)
            child_iter2 = self.node_tree2.iter_children(parent_iter2)
            while child_iter2 is not None:
                self.node_tree2[child_iter2][1] = current_value
                child_iter2 = self.node_tree2.iter_next(child_iter2)
        elif len(path) != 1:
            child_iter2 = self.node_tree2.get_iter(path)
            parent_iter2 = self.node_tree2.iter_parent(child_iter2)
            child_iter2 = self.node_tree2.iter_children(parent_iter2)

            all_selected = True
            while child_iter2 is not None:
                if self.node_tree2[child_iter2][1] == False:
                    all_selected = False
                    break
                child_iter2 = self.node_tree2.iter_next(child_iter2)

            self.node_tree2[parent_iter2][1] = all_selected

class MyApplication(Gtk.Application):

    def __init__(self):
        Gtk.Application.__init__(self)
        
    def do_activate(self):
        self.win = MyWindow(self)
        self.win.show_all()

    def do_startup(self):
        Gtk.Application.do_startup(self)

app = MyApplication()
exit_status = app.run(sys.argv)
app.win.doSystemExit()
# print(exit_status)
sys.exit(exit_status)