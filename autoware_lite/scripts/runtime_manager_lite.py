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
        self.alive_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.param_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        #self.param_Lchildbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        #self.param_Rchildbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.alive_label = Gtk.Label('alive label')
        self.alive_box.pack_start(self.alive_label, True, True, 0)

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

        self.param_Lframe.add(self.alive_box)
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

        alive_thread = threading.Thread(target=self.setAliveNodesState)
        alive_thread.daemon = True
        alive_thread.start()

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

    def setAliveNodesState(self):
        seq = 0
        while 1:
            detect_txt = 'Detection : \n' \
                            + '     LiDAR detector ---------------> ' + 'True' + '\n'\
                            + '     camera detector -------------> ' + 'True' + '\n'\
                            + '     lidar_kf_contour_track --> ' + 'True' + '\n'\
                            + '     lidar camera fusion --------> ' + 'True' + '\n' 
            follower_txt = 'Follower : \n' \
                            + '     twist filter ---------------> ' + 'True' + '\n'\
                            + '     pure pursuit ------------> ' + 'True' + '\n'\
                            + '     mpc --------------------------> ' + 'True' + '\n'\
                            + '     hybride stenly ---------> ' + 'True' + '\n'
            localizer_txt = 'Localizer : \n' \
                            + '     ndt matching --------> ' + 'True' + '\n'\
                            + '     ekf localizer -----------> ' + 'True' + '\n'
            lanechange_txt = 'LaneChange Manager : \n' \
                            + '     lanechange manager ------> ' + 'True' + '\n'
            local_plan_txt = 'Local Planner : \n' \
                            + '     op_common_params ---------> ' + 'True' + '\n'\
                            + '     op_trajectory_generator --> ' + 'True' + '\n'\
                            + '     op_motion_predictor --------> ' + 'True' + '\n'\
                            + '     op_trajectory_evaluator ---> ' + 'True' + '\n'\
                            + '     op_behavior_selector -------> ' + 'True' + '\n'
            vehicle_txt = 'Vehicle Setting : \n' \
                            + '     vel_pose_connect --------> ' + 'True' + '\n'\
                            + '     baselink to localizer -----> ' + 'True' + '\n'
            
            txt = detect_txt + follower_txt + localizer_txt + lanechange_txt + local_plan_txt + vehicle_txt
            self.alive_label.set_text(txt)
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
                print(instruction[idx1][idx2])
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
                print(instruction2[idx1][idx2])
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

