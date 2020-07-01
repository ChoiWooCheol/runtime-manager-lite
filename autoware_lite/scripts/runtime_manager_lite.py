#!/usr/bin/env python3

import os
import sys
import threading
import time
import rospy
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib
from gi.repository import Pango


nodes = [["Detection", ["LiDAR detector", False], ["camera detector", False], ["lidar_kf_contour_track", False], ["lidar camera fusion", False]],
         ["Follower", ["twist filter", False], ["pure pursuit", False], ["mpc", False], ["hybride stenly", False]],
         ["Localizer", ["ndt matching", False], ["ekf localizer", False]],
         ["Decision Maker", ["decision maker", False]],
         ["LaneChange Manager", ["lanechange manager", False]],
         ["Local Planner", ["op_common_params", False], ["op_trajectory_generator", False], ["op_motion_predictor", False],
            ["op_trajectory_evaluator", False],["op_behavior_selector", False]],
         ["Vehicle Setting", ["vel_pose_connect", False], ["baselink to localizer", False]]]

instruction = [["roslaunch lidar_detect lidar_detect.launch", "roslaunch camera_detect camera_detect.launch",  "roslaunch vehicle_set lidar_kf_contour_track.launch", "roslaunch range_vision_fusion range_vision_fusion.launch"],
               ["roslaunch follower pure_pursuit.launch", "roslaunch follower mpc.launch", "roslaunch follower hybride_stenly.launch"],
               ["roslaunch ndt_matching ndt_matching.launch", "roslaunch ekf_localizer ekf_localizer.launch"],
               ["roslaunch decision_maker decision_maker.launch"],
               ["roslaunch lanechange_manager lanechange_manager.launch"],
               ["roslaunch local_planner op_common_params.launch", "roslaunch local_planner op_trajectory_generator.launch", "roslaunch local_planner op_motion_predictor.launch",
               "roslaunch local_planner op_trajectory_evaluator.launch", "roslaunch local_planner op_behavior_selector.launch"],
               ["roslaunch vehicle_set vel_pose_connect.launch", "roslauch base2lidar base2lidar_tf.launch"]]

map_nodes = [["Map", ["point cloud", False], ["vector map", False], ["point_vector tf", False]],
             ["Sensing", ["sensor1", False], ["sensor2", False], ["sensor3", False], ["sensor4", False]],
             ["Point Downsampler", ["voxel grid filter", False]]]

instruction2 = [["rosrun map_file points_map_loader", "rosrun map_file vector_map_loader", "roslaunch /path/tf.launch"],
                ["roslaunch sensor_pkg sensor1.launch", "roslaunch sensor_pkg sensor2.launch", "roslaunch sensor_pkg sensor3.launch", "roslaunch sensor_pkg sensor4.launch"],
                ["roslaunch points_downsampler points_downsample.launch"]]


class MyWindow(Gtk.ApplicationWindow):

    def __init__(self, app):
        rospy.init_node('runime_manager_lite', anonymous=True)

        Gtk.Window.__init__(self, title="Autoware Lite", application=app)
        self.set_default_size(800, 600)
        self.set_border_width(20)

        autoware_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        #self.add(autoware_box)
        # the data are stored in the model
        # create a treestore with two columns
        self.store = Gtk.TreeStore(str, bool)
        # fill in the model
        for i in range(len(nodes)):
            # the iter piter is returned when appending the author in the first column
            # and False in the second
            piter = self.store.append(None, [nodes[i][0], False])
            # append the nodes and the associated boolean value as children of
            # the author
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
        renderer_in_out.connect("toggled", self.on_toggled)

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
        renderer_in_out2.connect("toggled", self.on_toggled2)
        
        
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

        self.timeout_id = GLib.timeout_add(50, self.on_timeout, None)
        
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
        self.param_Rframe.set_label("alive parameters")

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
        component_combo.connect("changed", self.on_component_combo_changed)
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
        mem_thread = threading.Thread(target=self.set_text, args=(self.mem_box,))
        mem_thread.daemon = True
        mem_thread.start()

        cpu_thread = threading.Thread(target=self.set_cpu_text, args=(self.cpu_box,))
        cpu_thread.daemon = True
        cpu_thread.start()
        
        disk_thread = threading.Thread(target=self.get_disk_space, args=(self.disk_box,))
        disk_thread.daemon = True
        disk_thread.start()

        stack = Gtk.Stack()
        stack.add_titled(autoware_box, 'child1', 'Autoware Nodes')  
        stack.add_titled(parameter_box, 'child2', 'Parameter Check')  
        stack.add_titled(resource_box, 'child3', 'Resource Check')  

        stack_switcher = Gtk.StackSwitcher(stack=stack) 

        header_bar = Gtk.HeaderBar(custom_title=stack_switcher, show_close_button=True) 

        
        self.set_titlebar(header_bar) 
        self.add(stack)

        #total = 0
        #while 1:
        #    time.sleep(1)
        #    total = total + 1
        #    txt = str(total) + 'resource check'
        #    self.mem_box.pack_start(Gtk.Label(txt), True, True, 0)
        # add the treeview to the window
        # self.add(view)


    def on_component_combo_changed(self, combo):
        tree_iter = combo.get_active_iter()
        if tree_iter is not None:
            model = combo.get_model()
            row_id, name = model[tree_iter][:2]
            #print("Selected: ID=%d, name=%s" % (row_id, name))
            self.alive_view_idx = row_id

            detect_txt = 'Detection : \n' \
                            + '     LiDAR detector ---------------> ' + 'True' + '\n'\
                            + '     camera detector -------------> ' + 'True' + '\n'\
                            + '     lidar_kf_contour_track --> ' + 'True' + '\n'\
                            + '     lidar camera fusion --------> ' + 'True' + '\n\n' 
            follower_txt = 'Follower : \n' \
                            + '     twist filter ---------------> ' + 'True' + '\n'\
                            + '     pure pursuit ------------> ' + 'True' + '\n'\
                            + '     mpc --------------------------> ' + 'True' + '\n'\
                            + '     hybride stenly ---------> ' + 'True' + '\n\n'
            localizer_txt = 'Localizer : \n' \
                            + '     ndt matching --------> ' + 'True' + '\n'\
                            + '     ekf localizer -----------> ' + 'True' + '\n\n'
            decision_txt = 'Decision Maker : \n'\
                            + '     decision maker --------->' + 'Ture' + '\n\n'
            lanechange_txt = 'LaneChange Manager : \n' \
                            + '     lanechange manager ------> ' + 'True' + '\n\n'
            local_plan_txt = 'Local Planner : \n' \
                            + '     op_common_params ---------> ' + 'True' + '\n'\
                            + '     op_trajectory_generator --> ' + 'True' + '\n'\
                            + '     op_motion_predictor --------> ' + 'True' + '\n'\
                            + '     op_trajectory_evaluator ---> ' + 'True' + '\n'\
                            + '     op_behavior_selector -------> ' + 'True' + '\n\n'
            vehicle_txt = 'Vehicle Setting : \n' \
                            + '     vel_pose_connect --------> ' + 'True' + '\n'\
                            + '     baselink to localizer -----> ' + 'True' + '\n\n'
            map_txt = 'Map : \n' \
                            + '     point cloud -------------> ' + 'True' + '\n'\
                            + '     vector map -------------> ' + 'True' + '\n'\
                            + '     point_vector tf -------> ' + 'True' + '\n\n'
            sensing_txt = 'Sensing : \n'\
                            + '     sensor1 -------> ' + 'True' + '\n'\
                            + '     sensor2 -------> ' + 'True' + '\n'\
                            + '     sensor3 -------> ' + 'True' + '\n'\
                            + '     sensor4 -------> ' + 'True' + '\n\n'
            downsampler_txt = 'Point Downsampler : \n'\
                            + '     voxel grid filter -----> ' + 'True' + '\n\n'

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

    def on_timeout(self, user_data):
        """
        Update value on the progress bar
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

    def set_cpu_text(self, widget):
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
    
    def set_text(self, widget):
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

    def get_disk_space(self, widget):
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
            
    def get_mem_resource(self, widget):
        p = os.popen('free -h')
        i = 0
        while 1:
            i = i + 1
            line = p.readline()
            if i==2:
                widget.pack_start(Gtk.Label(line), True, True, 0)

    def get_param_setting_win(self, idx1, idx2):
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

        if idx1 == 0: # Detection
            if idx2 == 0: # lidar detector
                param1 = Gtk.Label('detect_range (25, 50)')
                param2 = Gtk.Label('segmentation rate')
                param3 = Gtk.Label('12345678')

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()

                #default values
                param_val1.set_text('25')
                param_val2.set_text('100')
                param_val3.set_text('222')
                
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)
#            elif idx2 == 1: # camera detector
#            elif idx2 == 2: # lidar kf contour track
#            elif idx2 == 3: # lidar camera fusion
        elif idx1 == 1: # Follower
            if idx2 == 0: # twist filter /config/twist_filter
                param1 = Gtk.Label('lateral_accel_limit (0.0 ~ 5.0)')
                param2 = Gtk.Label('lowpass_gain_linear_x (0.0 ~ 1.0)')
                param3 = Gtk.Label('lowapss_gain_angular_z (0.0 ~ 1.0)')

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()

                #default values
                param_val1.set_text('0.8')
                param_val2.set_text('0')
                param_val3.set_text('0')
                
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)
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

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()
                param_val5 = Gtk.Entry()
                param_val6 = Gtk.Entry()
                param_val7 = Gtk.Entry()
                param_val8 = Gtk.Entry()
                param_val9 = Gtk.Entry()

                #default values
                param_val1.set_text('0')
                param_val2.set_text('5')
                param_val3.set_text('4')
                param_val4.set_text('2')
                param_val5.set_text('4')
                param_val6.set_text('0')
                param_val7.set_text('0')
                param_val8.set_text('False')
                param_val9.set_text('True')

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val5, param_val4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val6, param_val5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val7, param_val6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val8, param_val7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val9, param_val8, Gtk.PositionType.BOTTOM, 1, 1)
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

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()
                param_val5 = Gtk.Entry()
                param_val6 = Gtk.Entry()
                param_val7 = Gtk.Entry()
                param_val8 = Gtk.Entry()
                param_val9 = Gtk.Entry()
                param_val10 = Gtk.Entry()
                param_val11 = Gtk.Entry()
                param_val12 = Gtk.Entry()
                param_val13 = Gtk.Entry()
                param_val14 = Gtk.Entry()
                param_val15 = Gtk.Entry()
                param_val16 = Gtk.Entry()
                param_val17 = Gtk.Entry()
                param_val18 = Gtk.Entry()
                param_val19 = Gtk.Entry()
                param_val20 = Gtk.Entry()
                param_val21 = Gtk.Entry()
                param_val22 = Gtk.Entry()
                param_val23 = Gtk.Entry()
                param_val24 = Gtk.Entry()
                param_val25 = Gtk.Entry()
                param_val26 = Gtk.Entry()

                #default values
                param_val1.set_text('False') # show_debug_info
                param_val2.set_text('False') # publish_debug_values
                param_val3.set_text('kinematics') # vehicle_model_type
                param_val4.set_text('unconstraint_fast') # qp_solver_type
                param_val5.set_text('0.03') # ctrl_period
                param_val6.set_text('5') # admisible_position_error
                param_val7.set_text('90') # admisible_yaw_error_deg
                param_val8.set_text('70') # mpc_prediction_horizon
                param_val9.set_text('0.1') # mpc_prediction_sampling_time
                param_val10.set_text('0.1') # mpc_weight_lat_error
                param_val11.set_text('10') # mpc_weight_terminal_lat_error
                param_val12.set_text('0') # mpc_weight_heading_error
                param_val13.set_text('0.3') #  mpc_weight_heading_error_squared_vel_coeff
                param_val14.set_text('0.1') # mpc_weight_terminal_heading_error
                param_val15.set_text('0') # mpc_weight_lat_jerk
                param_val16.set_text('1') # mpc_weight_steering_input 
                param_val17.set_text('0.25') # mpc_weight_steering_input_squared_vel_coeff 
                param_val18.set_text('2') # mpc_zero_ff_steer_deg
                param_val19.set_text('True') # enable_path_smoothing
                param_val20.set_text('1') # path_smoothing_times 
                param_val21.set_text('35') # path_filter_moving_ave_num 
                param_val22.set_text('35') #curvature_smoothing_num 
                param_val23.set_text('3') #  steering_lpf_cutoff_hz 
                param_val24.set_text('0.3') #  vehicle_model_steer_tau 
                param_val25.set_text('2.9') # vehicle_model_wheelbase
                param_val26.set_text('35') # steer_lim_deg

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

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val5, param_val4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val6, param_val5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val7, param_val6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val8, param_val7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val9, param_val8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val10, param_val9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val11, param_val10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val12, param_val11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val13, param_val12, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val14, param_val13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val15, param_val14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val16, param_val15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val17, param_val16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val18, param_val17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val19, param_val18, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val20, param_val19, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val21, param_val20, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val22, param_val21, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val23, param_val22, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val24, param_val23, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val25, param_val24, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val26, param_val25, Gtk.PositionType.BOTTOM, 1, 1)
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
                param7 = Gtk.Label('max_iterations (1.0 ~ 300.0)')
                param8 = Gtk.Label('method_type (pcl_generic : 0, pcl_anh : 1, pcl_anh_gpu : 2, pcl_openmp : 3)')
                # launch param
                param9 = Gtk.Label('use_odom')
                param10 = Gtk.Label('use_imu')
                param11 = Gtk.Label('imu_upside_down')
                param12 = Gtk.Label('imu_topic')
                param13 = Gtk.Label('get_height')
                param14 = Gtk.Label('output_log_data')

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()
                param_val5 = Gtk.Entry()
                param_val6 = Gtk.Entry()
                param_val7 = Gtk.Entry()
                param_val8 = Gtk.Entry()
                param_val9 = Gtk.Entry()
                param_val10 = Gtk.Entry()
                param_val11 = Gtk.Entry()
                param_val12 = Gtk.Entry()
                param_val13 = Gtk.Entry()
                param_val14 = Gtk.Entry()

                #default values
                param_val1.set_text('1') # init_pos_gnss
                param_val2.set_text('0') # use_predict_pose
                param_val3.set_text('1') # error_threshold
                param_val4.set_text('1') # resolution
                param_val5.set_text('0.1') # step_size
                param_val6.set_text('0.01') # trans_epsilon
                param_val7.set_text('30') # max_iterations
                param_val8.set_text('0') # method_type
                param_val9.set_text('False') # use_odom
                param_val10.set_text('False') # use_imu
                param_val11.set_text('False') # imu_upside_down
                param_val12.set_text('/imu_raw') # imu_topic
                param_val13.set_text('False') # get_height
                param_val14.set_text('False') # output_log_data

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

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val5, param_val4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val6, param_val5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val7, param_val6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val8, param_val7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val9, param_val8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val10, param_val9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val11, param_val10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val12, param_val11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val13, param_val12, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val14, param_val13, Gtk.PositionType.BOTTOM, 1, 1)
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
                
                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()
                param_val5 = Gtk.Entry()
                param_val6 = Gtk.Entry()
                param_val7 = Gtk.Entry()
                param_val8 = Gtk.Entry()
                param_val9 = Gtk.Entry()
                param_val10 = Gtk.Entry()
                param_val11 = Gtk.Entry()
                param_val12 = Gtk.Entry()
                param_val13 = Gtk.Entry()

                #default values
                param_val1.set_text('False') # auto_mission_reload
                param_val2.set_text('False') # auto_engage
                param_val3.set_text('False') # auto_mission_change
                param_val4.set_text('False') # use_fms
                param_val5.set_text('True') # disuse_vector_map
                param_val6.set_text('20') # num_of_steer_behind
                param_val7.set_text('3') # goal_threshold_dist
                param_val8.set_text('0.1') # goal_threshold_vel
                param_val9.set_text('1') # change_threshold_dist
                param_val10.set_text('15') # change_threshold_angle
                param_val11.set_text('0.1') # stopped_vel
                param_val12.set_text('/points_lanes') # points_topic
                param_val13.set_text('base_link') # baselink_tf

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

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val5, param_val4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val6, param_val5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val7, param_val6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val8, param_val7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val9, param_val8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val10, param_val9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val11, param_val10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val12, param_val11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val13, param_val12, Gtk.PositionType.BOTTOM, 1, 1)                
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

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()
                param_val5 = Gtk.Entry()
                param_val6 = Gtk.Entry()
                param_val7 = Gtk.Entry()
                param_val8 = Gtk.Entry()
                param_val9 = Gtk.Entry()
                param_val10 = Gtk.Entry()
                param_val11 = Gtk.Entry()
                param_val12 = Gtk.Entry()
                param_val13 = Gtk.Entry()
                param_val14 = Gtk.Entry()
                param_val15 = Gtk.Entry()
                param_val16 = Gtk.Entry()
                param_val17 = Gtk.Entry()
                param_val18 = Gtk.Entry()
                param_val19 = Gtk.Entry()

                #default values
                param_val1.set_text('120') # horizonDistance
                param_val2.set_text('60') # maxLocalPlanDistance
                param_val3.set_text('0.5') # pathDensity
                param_val4.set_text('0.5') # rollOutDensity
                param_val5.set_text('4') # rollOutsNumber
                param_val6.set_text('15') # maxVelocity
                param_val7.set_text('5') # maxAcceleration
                param_val8.set_text('-3') # maxDeceleration
                param_val9.set_text('True') # enableFollowing
                param_val10.set_text('True') # enableSwerving
                param_val11.set_text('30') # minFollowingDistance
                param_val12.set_text('15') # minDistanceToAvoid
                param_val13.set_text('4') # maxDistanceToAvoid
                param_val14.set_text('True') # enableStopSignBehavior
                param_val15.set_text('False') # enableTrafficLightBehavior
                param_val16.set_text('False') # enableLaneChange 
                param_val17.set_text('0.5') # horizontalSafetyDistance 
                param_val18.set_text('0.5') # verticalSafetyDistance
                param_val19.set_text('1') # velocitySource

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

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val5, param_val4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val6, param_val5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val7, param_val6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val8, param_val7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val9, param_val8, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val10, param_val9, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val11, param_val10, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val12, param_val11, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val13, param_val12, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val14, param_val13, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val15, param_val14, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val16, param_val15, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val17, param_val16, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val18, param_val17, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val19, param_val18, Gtk.PositionType.BOTTOM, 1, 1)
            elif idx2 == 1: # op trajectory generator
                param1 = Gtk.Label('samplingTipMargin (0.1 ~ 10.0)')
                param2 = Gtk.Label('samplingOutMargin (0.2 ~ 40.0)')

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()

                param_val1.set_text('10') # samplingTipMargin
                param_val2.set_text('15') # samplingOutMargin

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
            elif idx2 == 2: # op motion predictor
                # launch params
                param1 = Gtk.Label('enableCurbObstacles')
                param2 = Gtk.Label('enableGenrateBranches')
                param3 = Gtk.Label('max_distance_to_lane (0.0 ~ 10.0)')
                param4 = Gtk.Label('prediction_distance (1.0 ~ 75.0)')
                param5 = Gtk.Label('enableStepByStepSignal')
                param6 = Gtk.Label('enableParticleFilterPrediction')

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()
                param_val5 = Gtk.Entry()
                param_val6 = Gtk.Entry()

                #default values
                param_val1.set_text('True') # enableCurbObstacles
                param_val2.set_text('False') # enableGenrateBranches
                param_val3.set_text('2') # max_distance_to_lane
                param_val4.set_text('25') # prediction_distance
                param_val5.set_text('False') # enableStepByStepSignal
                param_val6.set_text('False') # enableParticleFilterPrediction

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val5, param_val4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val6, param_val5, Gtk.PositionType.BOTTOM, 1, 1)
            elif idx2 == 3: # op trajectory evaluator
                param1 = Gtk.Label('enablePrediction')
                param_val1 = Gtk.Entry()
                param_val1.set_text('True') # enablePrediction
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
#            elif idx2 == 4: # op behavior selector
        elif idx1 == 6: # Vehicle Setting
            if idx2 == 0: # vel pose connect
                # launch params
                param1 = Gtk.Label('topic_pose_stamped')
                param2 = Gtk.Label('topic_twist_stamped')
                param3 = Gtk.Label('sim_mode')

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()

                param_val1.set_text('/ndt_pose') # samplingTipMargin
                param_val2.set_text('/estimate_twist') # samplingOutMargin
                param_val3.set_text('True') # max_distance_to_lane

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)
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

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()
                param_val5 = Gtk.Entry()
                param_val6 = Gtk.Entry()
                param_val7 = Gtk.Entry()
                param_val8 = Gtk.Entry()
                param_val9 = Gtk.Entry()

                #default values
                param_val1.set_text('1.2') # x
                param_val2.set_text('0') # y
                param_val3.set_text('2') # z
                param_val4.set_text('0') # yaw
                param_val5.set_text('0') # pitch
                param_val6.set_text('0') # roll
                param_val7.set_text('/base_link') # frame_id
                param_val8.set_text('/velodyne') # child_frame_id
                param_val9.set_text('10') # period_in_ms

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param5, param4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param6, param5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param7, param6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param8, param7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param9, param8, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)                
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val5, param_val4, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val6, param_val5, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val7, param_val6, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val8, param_val7, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val9, param_val8, Gtk.PositionType.BOTTOM, 1, 1)

        setparam_win.show_all()
        # 체크 눌리면 파라미터 세팅하는 창 만들기 고민 하다가 집갔음.
        # 함수에서 명령어를 실행할건지 아니면 체크박스가 눌리면 불리는 콜백함수에서 실행할건지 고민 <- 리턴값이 애매해서 힘들듯
        #exec_button.connect("clicked", self.on_click_me_clicked)
        
    def get_param_setting_win2(self, idx1, idx2):
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

        if idx1 == 0: # Map
            if idx2 == 0: # point cloud
                param1 = Gtk.Label('PCD MAP path')
                param_val1 = Gtk.Entry()
                #default values
                param_val1.set_text('path')
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
            elif idx2 == 1: # vector map
                param1 = Gtk.Label('Vector MAP path ( path1 path2 path3 ... )')
                param_val1 = Gtk.Entry()
                #default values
                param_val1.set_text('path1 path2 path3 ...')
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
            elif idx2 == 2: # point vector tf
                param1 = Gtk.Label('tf launch path')
                param_val1 = Gtk.Entry()
                #default values
                param_val1.set_text('path')
                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
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

                param_val1 = Gtk.Entry()
                param_val2 = Gtk.Entry()
                param_val3 = Gtk.Entry()
                param_val4 = Gtk.Entry()

                param_val1.set_text('voxel_grid_filter') # node_name
                param_val2.set_text('/points_raw') # points_topic
                param_val3.set_text('2') # voxel_leaf_size
                param_val4.set_text('200') # measurement_range

                in_grid.attach(param1, 0, 0, 1, 1)
                in_grid.attach_next_to(param2, param1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param3, param2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param4, param3, Gtk.PositionType.BOTTOM, 1, 1)

                in_grid.attach_next_to(param_val1, param1, Gtk.PositionType.RIGHT, 1, 1)
                in_grid.attach_next_to(param_val2, param_val1, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val3, param_val2, Gtk.PositionType.BOTTOM, 1, 1)
                in_grid.attach_next_to(param_val4, param_val3, Gtk.PositionType.BOTTOM, 1, 1)

        setparam_win.show_all()
    
    # def kill_node(self, idx1, idx2): # set free state of seleted node
    # def system_estop(self): # if system has estop state, it save all nodes info before kill all nodes
    # def rerun_routine(self): # do rerun routine, using saved all nodes info


        # callback function for the signal emitted by the cellrenderertoggle
    def on_toggled(self, widget, path):
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
                self.get_param_setting_win(idx1,idx2)
            else :
                print(instruction[idx1][idx2] + ' set false')

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
    
    def on_toggled2(self, widget, path):
        current_value = self.store2[path][1]
        self.store2[path][1] = not current_value
        current_value = not current_value
        path_idx = path.split(':')
        
        if len(path_idx) == 2:
            idx1 = int(path_idx[0])
            idx2 = int(path_idx[1])
            if current_value == True:
                # print(instruction2[idx1][idx2])
                self.get_param_setting_win2(idx1,idx2)
            else :
                print(instruction2[idx1][idx2] + ' set false')

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
        win = MyWindow(self)
        win.show_all()

    def do_startup(self):
        Gtk.Application.do_startup(self)

app = MyApplication()
exit_status = app.run(sys.argv)
print('exit')
sys.exit(exit_status)