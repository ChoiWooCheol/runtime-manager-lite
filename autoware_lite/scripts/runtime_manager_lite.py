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

nodes = [["Detection", ["LiDAR detector", False], ["camera detector", False], ["lidar_kf_contour_track", False], ["lidar camera fusion", False]],
         ["Follower", ["twist filter", False], ["pure pursuit", False], ["mpc", False], ["hybride stenly", False]],
         ["Localizer", ["ndt matching", False], ["ekf localizer", False]],
         ["Decision Maker", ["decision maker", False]],
         ["LaneChange Manager", ["lanechange manager", False]],
         ["Local Planner", ["op_common_params", False], ["op_trajectory_generator", False], ["op_motion_predictor", False],
            ["op_trajectory_evaluator", False],["op_behavior_selector", False]],
         ["Vehicle Setting", ["vel_pose_connect", False], ["baselink to localizer", False]]]

kill_instruction = [["/obb_generator /qt_detect_node", "/vision_darknet_detect /yolo3_rects",  "/lidar_kf_contour_track", "/detection/fusion_tools/range_fusion_visualization_01 /range_vision_fusion_01"],
               ["/twist_filter /twist_gate", "/pure_pursuit", "/mpc_follower /mpc_waypoints_converter", "/hybride_stenly"],
               ["/ndt_matching", "/ekf_localizer"],
               ["/decision_maker"],
               ["/lanechange_manager"],
               ["/op_common_params", "/op_trajectory_generator", "/op_motion_predictor", "/op_trajectory_evaluator", "/op_behavior_selector"],
               ["/pose_relay /vel_relay", "/base_link_to_localizer"]]

map_nodes = [["Map", ["point cloud", False], ["vector map", False], ["point_vector tf", False]],
             ["Sensing", ["sensor1", False], ["sensor2", False], ["sensor3", False], ["sensor4", False]],
             ["Point Downsampler", ["voxel grid filter", False]],
             ["QUICK START", ["map_quick_start", False], ["sensing_quick_start", False]]]

kill_instruction2 = [["/points_map_loader", "/vector_map_loader", "/world_to_map"],
                ["/sensor1", "/sensor2", "/sensor3", "/sensor4"],
                ["/voxel_grid_filter"]]

node_sequence_list = [] 
inst_sequence_list = []

check_alive = [["", "", "", ""], # Detection
               ["", ""], # Follower
               ["", ""], # Localizer
               [""], # Decision maker
               [""], # LaneChange manager
               ["", "", "", ""], # Local Planner
               [""], # Vehicle Setting
               ["", ""], # Map
               ["", "", "", ""], # Sensing
               [""]] # Point Downsampler

estop_state = False

def getDirPath():
	return os.path.abspath(os.path.dirname(__file__)) + "/"

class AutowareConfigPublisher:
    def __init__(self):
        self.pub_twist_filter       = rospy.Publisher('/config/twist_filter', ConfigTwistFilter, latch=True, queue_size=10)
        self.pub_waypoint_follower  = rospy.Publisher('/config/waypoint_follower', ConfigWaypointFollower, latch=True, queue_size=10)
        self.pub_ndt                = rospy.Publisher('/config/ndt', ConfigNDT, latch=True, queue_size=10)
        self.pub_decision_maker     = rospy.Publisher('/config/decision_maker', ConfigDecisionMaker, latch=True, queue_size=10)
        self.pub_voxel_grid_filter  = rospy.Publisher('/config/voxel_grid_filter', ConfigVoxelGridFilter, latch=True, queue_size=10)

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

        self.deadCount = [[-1,-1,-1,-1], # Detection
                          [-1,-1], # Follower
                          [-1,-1], # Localizer
                          [-1], # Decision maker
                          [-1], # LaneChange manager
                          [-1,-1,-1,-1], # Local Planner
                          [-1], # Vehicle Setting
                          [-1,-1], # Map
                          [-1,-1,-1,-1], # Sensing
                          [-1]] # Point Downsampler

        self.mutex = threading.Lock()

        self.update_thread = threading.Thread(target=self.upDateAliveState)
        self.update_thread.daemon = True
        self.update_thread.start()
        self.reset_time = 5
    def changeAliveState(self):
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
            self.deadCount[7][0]   = self.reset_time
            self.mutex.release()
    def checkVectorMapLoader(self, data):
        if "/vector_map_loader" in node_sequence_list:
            self.mutex.acquire()
            self.deadCount[7][1]   = self.reset_time
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
                    #print('deadCount'+str(i)+str(j)+str(self.deadCount[i][j]))

            self.mutex.release()

class MyWindow(Gtk.ApplicationWindow):

    def __init__(self, app):
        rospy.init_node('runime_manager_lite', anonymous=True)

        Gtk.Window.__init__(self, title="Autoware Lite", application=app)
        self.set_default_size(800, 600)
        self.set_border_width(20)

        autoware_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.store = Gtk.TreeStore(str, bool)
        # fill in the model
        for i in range(len(nodes)):
            piter = self.store.append(None, [nodes[i][0], False])
            j = 1
            while j < len(nodes[i]):
                self.store.append(piter, nodes[i][j])
                j += 1
                 
        self.store2 =  Gtk.TreeStore(str, bool)
        for i in range(len(map_nodes)):
            piter2 = self.store2.append(None, [map_nodes[i][0], False])
            j = 1
            while j < len(map_nodes[i]):
                self.store2.append(piter2, map_nodes[i][j])
                j += 1
        
        # generate autoware checking class  
        self.config_pub = AutowareConfigPublisher()
        self.alive_node_check = AutowareAliveNodesCheck()
        
        # the treeview shows the model
        # create a treeview on the model self.store
        self.nodes_Lwin = Gtk.ScrolledWindow()
        self.nodes_Lwin.set_border_width(2)
        self.nodes_Lwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.nodes_Rwin = Gtk.ScrolledWindow()
        self.nodes_Rwin.set_border_width(2)
        self.nodes_Rwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        view = Gtk.TreeView()
        autoware_box.pack_start(self.nodes_Lwin, True, True, 0)
        self.nodes_Lwin.add(view)
        view.set_model(self.store)

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
        view2.set_model(self.store2)
        renderer_nodes2 = Gtk.CellRendererText()
        column_nodes2 = Gtk.TreeViewColumn("Map & Sensing", renderer_nodes2, text=0)
        view2.append_column(column_nodes2)
        renderer_in_out2 = Gtk.CellRendererToggle()
        column_in_out2 = Gtk.TreeViewColumn("Execute", renderer_in_out2, active=1)
        view2.append_column(column_in_out2)
        renderer_in_out2.connect("toggled", self.onToggled2)
        
        
        # Resource Check Frame
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
        
        # parametor stack setting
        parameter_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.alive_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.param_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        #self.param_Lchildbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        #self.param_Rchildbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        
        self.alive_label = Gtk.Label('Select Component')
        self.alive_box.pack_start(self.alive_label, True, True, 0)
        #self.alive_box.pack_start(button, True, True, 0)

        self.param_Lframe = Gtk.Frame()
        self.param_Rframe = Gtk.Frame()

        self.param_Lframe.set_shadow_type(1)
        self.param_Lframe.set_border_width(2)
        self.param_Lframe.set_label("alive nodes")

        self.param_Rframe.set_shadow_type(1)
        self.param_Rframe.set_border_width(2)
        self.param_Rframe.set_label("alive parameter")

        self.param_Lwin = Gtk.ScrolledWindow()
        self.param_Lwin.set_border_width(2)
        self.param_Lwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        self.param_Rwin = Gtk.ScrolledWindow()
        self.param_Rwin.set_border_width(2)
        self.param_Rwin.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)

        #self.param_Lframe.add(self.alive_box)

        self.grid = Gtk.Grid()
        self.grid.set_column_homogeneous(True)
        self.grid.set_row_homogeneous(True)
        self.param_Lframe.add(self.grid)
        self.grid.attach(self.alive_box, 0, 0, 1, 13)


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

        self.grid.attach_next_to(
            component_combo, self.alive_box, Gtk.PositionType.BOTTOM, 1, 1
        )
        
        self.param_Lwin.add(self.param_Lframe)
        self.param_Rwin.add(self.param_Rframe)

        parameter_box.pack_start(self.param_Lwin, True, True, 0)
        parameter_box.pack_start(self.param_Rwin, True, True, 0)
        
        #self.param_Lchildbox.pack_start(Gtk.Label("left"),True, True, 0)
        #self.param_Rchildbox.pack_start(Gtk.Label("right"),True, True, 0)
        self.mem_thread = threading.Thread(target=self.setText, args=(self.mem_box,))
        self.mem_thread.daemon = True
        self.mem_thread.start()

        self.cpu_thread = threading.Thread(target=self.setCpuText, args=(self.cpu_box,))
        self.cpu_thread.daemon = True
        self.cpu_thread.start()
        
        self.disk_thread = threading.Thread(target=self.getDistSpace, args=(self.disk_box,))
        self.disk_thread.daemon = True
        self.disk_thread.start()

        stack = Gtk.Stack()
        stack.add_titled(autoware_box, 'child1', 'Autoware Nodes')  
        stack.add_titled(parameter_box, 'child2', 'Alive Check')  
        stack.add_titled(resource_box, 'child3', 'Resource Check')  

        stack_switcher = Gtk.StackSwitcher(stack=stack) 

        header_bar = Gtk.HeaderBar(custom_title=stack_switcher, show_close_button=True) 

        
        self.set_titlebar(header_bar) 
        self.add(stack)

    def onComponentComboChanged(self, combo):
        tree_iter = combo.get_active_iter()
        self.alive_node_check.changeAliveState()
        if tree_iter is not None:
            model = combo.get_model()
            row_id, name = model[tree_iter][:2]
            #print("Selected: ID=%d, name=%s" % (row_id, name))
            self.alive_view_idx = row_id

            detect_txt = 'Detection : \n' \
                            + '     LiDAR detector ---------------> ' + check_alive[0][0] + '\n'\
                            + '     camera detector -------------> '  + check_alive[0][1] + '\n'\
                            + '     lidar_kf_contour_track --> '      + check_alive[0][2] + '\n'\
                            + '     lidar camera fusion --------> '   + check_alive[0][3] + '\n\n' 
            follower_txt = 'Follower : \n' \
                            + '     twist filter ---------------> '   + check_alive[1][0] + '\n'\
                            + '     follower ------------> '          + check_alive[1][1] + '\n\n'
            localizer_txt = 'Localizer : \n' \
                            + '     ndt matching --------> '          + check_alive[2][0] + '\n'\
                            + '     ekf localizer -----------> '      + check_alive[2][1] + '\n\n'
            decision_txt = 'Decision Maker : \n'\
                            + '     decision maker --------->'        + check_alive[3][0] + '\n\n'
            lanechange_txt = 'LaneChange Manager : \n' \
                            + '     lanechange manager ------> '      + check_alive[4][0] + '\n\n'
            local_plan_txt = 'Local Planner : \n' \
                            + '     op_motion_predictor --------> '   + check_alive[5][0] + '\n'\
                            + '     op_trajectory_evaluator ---> '    + check_alive[5][1] + '\n'\
                            + '     op_trajectory_generator --> '     + check_alive[5][2] + '\n'\
                            + '     op_behavior_selector -------> '   + check_alive[5][3] + '\n\n'
            vehicle_txt = 'Vehicle Setting : \n' \
                            + '     vel_pose_connect --------> '      + check_alive[6][0] + '\n\n'
            map_txt = 'Map : \n' \
                            + '     point cloud -------------> '      + check_alive[7][0] + '\n'\
                            + '     vector map -------------> '       + check_alive[7][1] + '\n\n'
            sensing_txt = 'Sensing : \n'\
                            + '     sensor1 -------> ' + check_alive[8][0] + '\n'\
                            + '     sensor2 -------> ' + check_alive[8][1] + '\n'\
                            + '     sensor3 -------> ' + check_alive[8][2] + '\n'\
                            + '     sensor4 -------> ' + check_alive[8][3] + '\n\n'
            downsampler_txt = 'Point Downsampler : \n'\
                            + '     voxel grid filter -----> ' + check_alive[9][0] + '\n\n'

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
                txt = detect_txt + follower_txt + localizer_txt + decision_txt + \
                      lanechange_txt + local_plan_txt + vehicle_txt + map_txt + sensing_txt + downsampler_txt

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
        
        param_list = []
        if idx1 == 0: # Detection
            if idx2 == 0: # lidar detector
                param1 = Gtk.Label('detect_range (25, 50)')
                param2 = Gtk.Label('segmentation rate')
                param3 = Gtk.Label('12345678')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()
                
                #default values
                pv1.set_text('25')
                pv2.set_text('100')
                pv3.set_text('222')
                
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv2, pv1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(pv3, pv2, Gtk.PositionType.BOTTOM, 1, 1)

                param_list = [pv1,pv2,pv3]
#            elif idx2 == 1: # camera detector
#            elif idx2 == 2: # lidar kf contour track
#            elif idx2 == 3: # lidar camera fusion
        elif idx1 == 1: # Follower
            if idx2 == 0: # twist filter /config/twist_filter
                param1 = Gtk.Label('lateral_accel_limit (0.0 ~ 5.0)')
                param2 = Gtk.Label('lowpass_gain_linear_x (0.0 ~ 1.0)')
                param3 = Gtk.Label('lowpass_gain_angular_z (0.0 ~ 1.0)')

                pv1 = Gtk.Entry()
                pv2 = Gtk.Entry()
                pv3 = Gtk.Entry()

                #default values
                pv1.set_text('0.8')
                pv2.set_text('0')
                pv3.set_text('0')
                
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

                #default values
                pv1.set_text('0')
                pv2.set_text('5')
                pv3.set_text('4')
                pv4.set_text('2')
                pv5.set_text('4')
                pv6.set_text('0')
                pv7.set_text('0')
                pv8.set_text('False')
                pv9.set_text('True')

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

                #default values
                pv1.set_text('False') # show_debug_info
                pv2.set_text('False') # publish_debug_values
                pv3.set_text('kinematics') # vehicle_model_type
                pv4.set_text('unconstraint_fast') # qp_solver_type
                pv5.set_text('0.03') # ctrl_period
                pv6.set_text('5') # admisible_position_error
                pv7.set_text('90') # admisible_yaw_error_deg
                pv8.set_text('70') # mpc_prediction_horizon
                pv9.set_text('0.1') # mpc_prediction_sampling_time
                pv10.set_text('0.1') # mpc_weight_lat_error
                pv11.set_text('10') # mpc_weight_terminal_lat_error
                pv12.set_text('0') # mpc_weight_heading_error
                pv13.set_text('0.3') #  mpc_weight_heading_error_squared_vel_coeff
                pv14.set_text('0.1') # mpc_weight_terminal_heading_error
                pv15.set_text('0') # mpc_weight_lat_jerk
                pv16.set_text('1') # mpc_weight_steering_input 
                pv17.set_text('0.25') # mpc_weight_steering_input_squared_vel_coeff 
                pv18.set_text('2') # mpc_zero_ff_steer_deg
                pv19.set_text('True') # enable_path_smoothing
                pv20.set_text('1') # path_smoothing_times 
                pv21.set_text('35') # path_filter_moving_ave_num 
                pv22.set_text('35') #curvature_smoothing_num 
                pv23.set_text('3') #  steering_lpf_cutoff_hz 
                pv24.set_text('0.3') #  vehicle_model_steer_tau 
                pv25.set_text('2.9') # vehicle_model_wheelbase
                pv26.set_text('35') # steer_lim_deg

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

                #default values
                pv1.set_text('1') # init_pos_gnss
                pv2.set_text('0') # use_predict_pose
                pv3.set_text('1') # error_threshold
                pv4.set_text('1') # resolution
                pv5.set_text('0.1') # step_size
                pv6.set_text('0.01') # trans_epsilon
                pv7.set_text('30') # max_iterations
                pv8.set_text('0') # method_type
                pv9.set_text('False') # use_odom
                pv10.set_text('False') # use_imu
                pv11.set_text('False') # imu_upside_down
                pv12.set_text('/imu_raw') # imu_topic
                pv13.set_text('False') # get_height
                pv14.set_text('False') # output_log_data

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
                in_grid.attach_next_to(pv11, pv10, Gtk.PositionType.BOTTOM, 1, 1)
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

                #default values
                pv1.set_text('False') # auto_mission_reload
                pv2.set_text('False') # auto_engage
                pv3.set_text('False') # auto_mission_change
                pv4.set_text('False') # use_fms
                pv5.set_text('True') # disuse_vector_map
                pv6.set_text('20') # num_of_steer_behind
                pv7.set_text('3') # goal_threshold_dist
                pv8.set_text('0.1') # goal_threshold_vel
                pv9.set_text('1') # change_threshold_dist
                pv10.set_text('15') # change_threshold_angle
                pv11.set_text('0.1') # stopped_vel
                pv12.set_text('/points_lanes') # points_topic
                pv13.set_text('base_link') # baselink_tf

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
                param9 = Gtk.Label('enableFollowing)')
                param10 = Gtk.Label('enableSwerving')
                param11 = Gtk.Label('minFollowingDistance (0.5 ~ 100.0)')
                param12 = Gtk.Label('minDistanceToAvoid (0.1 ~ 80.0)')
                param13 = Gtk.Label('maxDistanceToAvoid (0.05 ~ 60.0)')
                param14 = Gtk.Label('enableStopSignBehavior')
                param15 = Gtk.Label('enableTrafficLightBehavior')
                param16 = Gtk.Label('enableLaneChange')
                param17 = Gtk.Label('horizontalSafetyDistance (0.0 ~ 10.0')
                param18 = Gtk.Label('verticalSafetyDistance (0.0 ~ 25.0')
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

                #default values
                pv1.set_text('120') # horizonDistance
                pv2.set_text('60') # maxLocalPlanDistance
                pv3.set_text('0.5') # pathDensity
                pv4.set_text('0.5') # rollOutDensity
                pv5.set_text('4') # rollOutsNumber
                pv6.set_text('15') # maxVelocity
                pv7.set_text('5') # maxAcceleration
                pv8.set_text('-3') # maxDeceleration
                pv9.set_text('True') # enableFollowing
                pv10.set_text('True') # enableSwerving
                pv11.set_text('30') # minFollowingDistance
                pv12.set_text('15') # minDistanceToAvoid
                pv13.set_text('4') # maxDistanceToAvoid
                pv14.set_text('True') # enableStopSignBehavior
                pv15.set_text('False') # enableTrafficLightBehavior
                pv16.set_text('False') # enableLaneChange 
                pv17.set_text('0.5') # horizontalSafetyDistance 
                pv18.set_text('0.5') # verticalSafetyDistance
                pv19.set_text('1') # velocitySource

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

                pv1.set_text('10') # samplingTipMargin
                pv2.set_text('15') # samplingOutMargin

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

                #default values
                pv1.set_text('True') # enableCurbObstacles
                pv2.set_text('False') # enableGenrateBranches
                pv3.set_text('2') # max_distance_to_lane
                pv4.set_text('25') # prediction_distance
                pv5.set_text('False') # enableStepByStepSignal
                pv6.set_text('False') # enableParticleFilterPrediction

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
                pv1.set_text('True') # enablePrediction
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

                pv1.set_text('/ndt_pose') # samplingTipMargin
                pv2.set_text('/estimate_twist') # samplingOutMargin
                pv3.set_text('True') # max_distance_to_lane

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

                #default values
                pv1.set_text('1.2') # x
                pv2.set_text('0') # y
                pv3.set_text('2') # z
                pv4.set_text('0') # yaw
                pv5.set_text('0') # pitch
                pv6.set_text('0') # roll
                pv7.set_text('/base_link') # frame_id
                pv8.set_text('/velodyne') # child_frame_id
                pv9.set_text('10') # period_in_ms

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
        
        grid.attach_next_to(
            exec_button, set_param_box, Gtk.PositionType.BOTTOM, 1, 1
        )
        set_param_box.pack_start(in_grid, True, True, 0)
        param_list = []
        if idx1 == 0: # Map
            if idx2 == 0: # point cloud
                param1 = Gtk.Label('PCD MAP path')
                pv1 = Gtk.Entry()
                #default values
                pv1.set_text('path')
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                param_list = [pv1]
            elif idx2 == 1: # vector map
                param1 = Gtk.Label('Vector MAP path ( path1 path2 path3 ... )')
                pv1 = Gtk.Entry()
                #default values
                pv1.set_text('path1 path2 path3 ...')
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                param_list = [pv1]
            elif idx2 == 2: # point vector tf
                param1 = Gtk.Label('tf launch path')
                pv1 = Gtk.Entry()
                #default values
                pv1.set_text('path')
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(pv1, param1, Gtk.PositionType.RIGHT, 1, 1)
                param_list = [pv1]
        #elif idx1 == 1: # Sensing
            #if idx2 == 0: # sensor1
            #elif idx2 == 1: # sensor2
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

                pv1.set_text('voxel_grid_filter') # node_name
                pv2.set_text('/points_raw') # points_topic
                pv3.set_text('2') # voxel_leaf_size
                pv4.set_text('200') # measurement_range

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
                #run_cmd = ('roslaunch vision_darknet_detect vision_yolo3_detect.launch'+
                #            'score_threshold:=' + plist[].get_text()
                #            + 'nms_threshold:' + plist[].get_text()
                #            + 'image_src:' + plist[].get_text()
                #            + 'network_definition_file:=' + plist[].get_text() 
                #            + 'pretrained_model_file:=' + plist[].get_text()
                #            + 'names_file:=' + plist[].get_text()
                #            + 'gpu_device_id:' + plist[].get_text()
                #            + 'camera_id:=' + plist[].get_text())
                node_sequence_list.append('/vision_darknet_detect /yolo3_rects')
            elif idx2 == 2: # lidar kf contour track
                run_cmd = 'None'
                node_sequence_list.append('/lidar_kf_contour_track')
            elif idx2 == 3: # lidar camera fusion
                run_cmd = 'None'
                node_sequence_list.append('/detection/fusion_tools/range_fusion_visualization_01 /range_vision_fusion_01')
        elif idx1 == 1: # Follower
            if idx2 == 0: # twist filter /config/twist_filter
                data = ConfigTwistFilter()
                data.lateral_accel_limit    = float(plist[0].get_text())
                data.lowpass_gain_linear_x  = float(plist[1].get_text())
                data.lowpass_gain_angular_z = float(plist[2].get_text())
                self.config_pub.onConfigTwistFilter(data)
                run_cmd = 'roslaunch waypoint_follower twist_filter.launch'
                node_sequence_list.append('/twist_filter /twist_gate')
            elif idx2 == 1: # pure pursuit /config/waypoint_follwer
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
                          + ' show_debug_info:=' + plist[0].get_text() 
                          + ' publish_debug_values:=' + plist[1].get_text() 
                          + ' vehicle_model_type:=' + plist[2].get_text() 
                          + ' qp_solver_type:=' + plist[3].get_text() 
                          + ' admisible_position_error:=' + plist[4].get_text() 
                          + ' admisible_yaw_error_deg:=' + plist[5].get_text() 
                          + ' mpc_prediction_horizon:=' + plist[6].get_text() 
                          + ' mpc_prediction_sampling_time:=' + plist[7].get_text() 
                          + ' mpc_weight_lat_error:=' + plist[8].get_text() 
                          + ' mpc_weight_terminal_lat_error:=' + plist[9].get_text() 
                          + ' mpc_weight_heading_error:=' + plist[10].get_text() 
                          + ' mpc_weight_heading_error_squared_vel_coeff:=' + plist[11].get_text() 
                          + ' mpc_weight_terminal_heading_error:=' + plist[12].get_text() 
                          + ' mpc_weight_lat_jerk:=' + plist[13].get_text() 
                          + ' mpc_weight_steering_input:=' + plist[14].get_text() 
                          + ' mpc_weight_steering_input_squared_vel_coeff:=' + plist[15].get_text() 
                          + ' mpc_zero_ff_steer_deg:=' + plist[16].get_text() 
                          + ' enable_path_smoothing:=' + plist[17].get_text() 
                          + ' path_smoothing_times:=' + plist[18].get_text() 
                          + ' path_filter_moving_ave_num:=' + plist[19].get_text() 
                          + ' curvature_smoothing_num:=' + plist[20].get_text() 
                          + ' steering_lpf_cutoff_hz:=' + plist[21].get_text() 
                          + ' vehicle_model_steer_tau:=' + plist[22].get_text() 
                          + ' vehicle_model_wheelbase:=' + plist[23].get_text() 
                          + ' steer_lim_deg:=' + plist[24].get_text())
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
                + ' method_type:=' + plist[7].get_text() 
                + ' use_odom:=' + plist[8].get_text() 
                + ' use_imu:=' + plist[9].get_text() 
                + ' imu_upside_down:=' + plist[10].get_text() 
                + ' imu_topic:=' + plist[11].get_text() 
                + ' get_height:=' + plist[12].get_text() 
                + ' output_log_data:=' + plist[13].get_text())
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
                          + ' disuse_vector_map:=' + plist[4].get_text() 
                          + ' points_topic:=' + plist[11].get_text() 
                          + ' baselink_tf:=' + plist[12].get_text() 
                          + ' use_fms:=' + plist[3].get_text() 
                          + ' auto_mission_reload:=' + plist[0].get_text() 
                          + ' auto_engage:=' + plist[1].get_text() 
                          + ' auto_mission_change:=' + plist[2].get_text())
                node_sequence_list.append('/decision_maker')
        elif idx1 == 4: # LaneChange Manager
            if idx2 == 0: # lanechange manager
                run_cmd = 'roslaunch lanechange_manager lanechange_manager.launch'
                node_sequence_list.append('/lanechange_manager')
        elif idx1 == 5: # Local Planner
            if idx2 == 0: # op commom params
                run_cmd = ('roslaunch op_local_planner op_common_params.launch horizonDistance:=' + plist[0].get_text() 
                          + ' maxLocalPlanDistance:=' + plist[1].get_text() 
                          + ' pathDensity:='+ plist[2].get_text() 
                          + ' rollOutDensity:=' + plist[3].get_text() 
                          + ' rollOutsNumber:=' + plist[4].get_text() 
                          + ' maxVelocity:=' + plist[5].get_text() 
                          + ' maxAcceleration:=' + plist[6].get_text() 
                          + ' maxDeceleration:=' + plist[7].get_text() 
                          + ' enableFollowing:=' + plist[8].get_text() 
                          + ' enableSwerving:=' + plist[9].get_text() 
                          + ' minFollowingDistance:=' + plist[10].get_text() 
                          + ' minDistanceToAvoid:=' + plist[11].get_text() 
                          + ' maxDistanceToAvoid:=' + plist[12].get_text() 
                          + ' enableStopSignBehavior:=' + plist[13].get_text() 
                          + ' enableTrafficLightBehavior:=' + plist[14].get_text() 
                          + ' enableLaneChange:=' + plist[15].get_text() 
                          + ' horizontalSafetyDistance:=' + plist[16].get_text() 
                          + ' verticalSafetyDistance:=' + plist[17].get_text() 
                          + ' velocitySource:=' + plist[18].get_text())
                node_sequence_list.append('/op_common_params')
            elif idx2 == 1: # op trajectory generator
                run_cmd = ('roslaunch op_local_planner op_trajectory_generator.launch'
                          + ' samplingTipMargin:=' + plist[0].get_text() 
                          + ' samplingOutMargin:=' + plist[1].get_text())
                node_sequence_list.append('/op_trajectory_generator')
            elif idx2 == 2: # op motion predictor
                run_cmd = ('roslaunch op_local_planner op_motion_predictor.launch enableCurbObstacles:=' + plist[0].get_text() 
                          + ' enableGenrateBranches:=' + plist[1].get_text() 
                          + ' max_distance_to_lane:=' + plist[2].get_text() 
                          + ' prediction_distance:=' + plist[3].get_text() 
                          + ' enableStepByStepSignal:=' + plist[4].get_text() 
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
                        + ' topic_pose_stamped:=' + plist[0].get_text() 
                        + ' topic_twist_stamped:=' + plist[1].get_text() 
                        + ' sim_mode:=' + plist[2].get_text())
                node_sequence_list.append('/pose_relay /vel_relay')
            elif idx2 == 1: # baselink to localizer
                run_cmd = ('roslaunch runtime_manager setup_tf.launch'
                          + ' x:=' + plist[0].get_text() 
                          + ' y:=' + plist[1].get_text() 
                          + ' z:=' + plist[2].get_text() 
                          + ' yaw:=' + plist[3].get_text() 
                          + ' pitch:=' + plist[4].get_text() 
                          + ' roll:=' + plist[5].get_text() 
                          + ' frame_id:=' + plist[6].get_text() 
                          + ' child_frame_id:=' + plist[7].get_text() 
                          + ' period_in_ms:=' + plist[8].get_text())
                node_sequence_list.append('/base_link_to_localizer')
        
        if run_cmd == 'None':
            window.close()
        else:
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
        #elif idx1 == 1: # Sensing
            #if idx2 == 0: # sensor1
            #elif idx2 == 1: # sensor2
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
                self.mapQuickStart()
            elif idx2 == 1: # sensing quick start
                run_cmd = 'None'
                self.sensingQuickStart()
        if run_cmd == 'None':
            window.close()
        else:
            self.onExcuteThread(run_cmd)
            inst_sequence_list.append(run_cmd)
            window.close()

    def killNode(self, inst, idx1, idx2): # set free state of seleted node
        if inst == 1 and kill_instruction[idx1][idx2] in node_sequence_list:
            os.system("rosnode kill " + kill_instruction[idx1][idx2])
            del_idx = node_sequence_list.index(kill_instruction[idx1][idx2])
            del node_sequence_list[del_idx]
            del inst_sequence_list[del_idx]
        elif inst == 2 and kill_instruction2[idx1][idx2] in node_sequence_list:
            os.system("rosnode kill " + kill_instruction2[idx1][idx2])
            del_idx = node_sequence_list.index(kill_instruction2[idx1][idx2])
            del node_sequence_list[del_idx]
            del inst_sequence_list[del_idx]
    
    def mapQuickStart(self):
        dir_path = getDirPath()
        path = dir_path + 'map_quickstart.yaml'
        print('loading... ' + path)
        f = open(path, 'r')
        d = yaml.load(f)
        f.close()
        quick_list = d.get('routine', [])
        
        for inst in quick_list:
            cmd = inst['instruction']
            for par in  inst['parameters']:
                cmd = cmd + ' ' + par['args']
            print(cmd)

    def sensingQuickStart(self):
        dir_path = getDirPath()
        path = dir_path + 'sensing_quickstart.yaml'
        print('loading ' + path)
        f = open(path, 'r')
        d = yaml.load(f)
        f.close()
        quick_list = d.get('routine', [])
        
        for inst in quick_list:
            cmd = inst['instruction']
            for par in  inst['parameters']:
                cmd = cmd + ' ' + par['args']
            print(cmd)

    def doSystemExit(self): # if system has estop state, it save all nodes info before kill all nodes
        print("\nEXIT MANAGER! Killing all nodes .. ... ..")
        for i in range(len(node_sequence_list)):
            os.system("rosnode kill " + node_sequence_list[i])
        print("EXIT routine... Done")

    def doSystemEstop(self): # if system has estop state, it save all nodes info before kill all nodes
        print("\nE-STOP!! Killing all nodes .. ... ..")
        for i in range(len(node_sequence_list)):
            os.system("rosnode kill " + node_sequence_list[i])
        print("E-stop routine... Done")
        estop_state = True

    def doRerunRoutine(self): # do rerun routine, using saved all nodes info
        if estop_state == False:
            return

        print("\nDo rerun routine")
        print("----------- rerun node list -----------")
        for i in range(len(node_sequence_list)):
            print(node_sequence_list[i])
        print("---------------------------------------")

        for i in range(len(inst_sequence_list)):
            time.sleep(1)
            self.onExcuteThread(inst_sequence_list[i])

        estop_state = False

        # callback function for the signal emitted by the cellrenderertoggle
    def onToggled(self, widget, path):
        # the boolean value of the selected row
        current_value = self.store[path][1]
        # change the boolean value of the selected row in the model
        self.store[path][1] = not current_value
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
            piter = self.store.get_iter(path)
            # get the iter associated with its first child
            citer = self.store.iter_children(piter)
            # while there are children, change the state of their boolean value
            # to the value of the author
            while citer is not None:
                self.store[citer][1] = current_value
                citer = self.store.iter_next(citer)
        # if the length of the path is not 1 (that is, if we are selecting a
        # book)
        elif len(path) != 1:
            # get the first child of the parent of the book (the first book of
            # the author)
            citer = self.store.get_iter(path)
            piter = self.store.iter_parent(citer)
            citer = self.store.iter_children(piter)
            # check if all the children are selected
            all_selected = True
            while citer is not None:
                if self.store[citer][1] == False:
                    all_selected = False
                    break
                citer = self.store.iter_next(citer)
            # if they do, the author as well is selected; otherwise it is not
            self.store[piter][1] = all_selected
    
    def onToggled2(self, widget, path):
        current_value = self.store2[path][1]
        self.store2[path][1] = not current_value
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
            piter2 = self.store2.get_iter(path)
            citer2 = self.store2.iter_children(piter2)
            while citer2 is not None:
                self.store2[citer2][1] = current_value
                citer2 = self.store2.iter_next(citer2)
        elif len(path) != 1:
            citer2 = self.store2.get_iter(path)
            piter2 = self.store2.iter_parent(citer2)
            citer2 = self.store2.iter_children(piter2)

            all_selected = True
            while citer2 is not None:
                if self.store2[citer2][1] == False:
                    all_selected = False
                    break
                citer2 = self.store2.iter_next(citer2)

            self.store2[piter2][1] = all_selected

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
sys.exit(exit_status)