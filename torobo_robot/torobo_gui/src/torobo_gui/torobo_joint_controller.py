#!/usr/bin/env python
## -*- coding: utf-8 -*-


import os
import rospy
import rosparam
import rospkg
import numpy as np
import copy
from collections import OrderedDict

import tf
import tf2_ros
import moveit_commander
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

from joint_limits_urdf import get_joint_limits

from torobo_motion_manager import move_home_position_client
from torobo_motion_manager import teaching_point_manager
from torobo_motion_manager import teaching_trajectory_manager

from torobo_common import repeat_get_param

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

from ast import parse
import yaml

from control_msgs.msg import GripperCommand, GripperCommandActionGoal
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib_msgs.msg import GoalStatus


class ToroboJointController(Plugin):

    def __init__(self, context):

        super(self.__class__, self).__init__(context)
        self.setObjectName('ToroboJointController')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()

        # load ui
        ui_file = os.path.join(rospkg.RosPack().get_path('torobo_gui'), 'resource', 'torobo_joint_controller.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ToroboJointControllerUi')

        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self.serial_number = context.serial_number()

        # Create sub widgets
        self.create()

    def shutdown_plugin(self):
        self.destroy()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def create(self):

        # Init values
        self._nameSpace = rospy.get_namespace()
        if (self._nameSpace == "/"):
            self._nameSpace = "torobo"
        if self._nameSpace[0] == "/":
            self._nameSpace = self._nameSpace[1:]
        if self._nameSpace[-1] == "/":
            self._nameSpace = self._nameSpace[:-1]
        self._controller_info = self.get_controller_info("/torobo/move_group/controller_list")
        self._joint_limits = None
        self._double_editor_list = []
        self._trajectory_command_publisher = None
        self._display_planned_path_publisher = rospy.Publisher("/torobo/move_group/display_planned_path", DisplayTrajectory, queue_size=1)
        self._JOINT_STATE_DATA_ = JointState()
        self._current_tp = None

        # comboBox (ControllerNameSpace)
        self._widget.comboBoxControllerNamespace.addItem(self._nameSpace)

        # comboBox (ControllerName)
        for controller in self._controller_info.keys():
            self._widget.comboBoxControllerName.addItem(controller)
        self._widget.comboBoxControllerName.activated[str].connect(self.setup_joint_group)
        if (self.serial_number <= self._widget.comboBoxControllerName.count()):
            self._widget.comboBoxControllerName.setCurrentIndex(-1)

        # comboBox (TpSelect)
        self._comboBoxTpList = EnhancedComboBox()
        self._comboBoxTpList.addItem("<New>")
        self._comboBoxTpList.setCurrentIndex(-1)
        self._comboBoxTpList.activated[str].connect(self.setup_tp)
        self._comboBoxTpList.focusReleased.connect(self.focus_released_tp)
        self._comboBoxTpList.editingFinished.connect(self.edit_finish_tp)
        self._comboBoxTpList.setEnabled(False)
        self._comboBoxTpList.setEditable(False)
        self._widget.gridLayoutTP.addWidget(self._comboBoxTpList, 0, 0)

        # button (Save, Delete)
        self._widget.pushButtonSave.clicked.connect(self.pushButton_clicked)
        self._widget.pushButtonSave.setEnabled(False)
        self._widget.pushButtonDelete.clicked.connect(self.pushButton_clicked)
        self._widget.pushButtonDelete.setEnabled(False)

        # button (FileSave, FileOpen)
        self._widget.pushButtonFileSave.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.pushButtonFileSave.clicked.connect(self.pushButton_clicked)
        self._widget.pushButtonFileSave.setEnabled(False)
        self._widget.pushButtonFileOpen.setIcon(QIcon.fromTheme('document-open'))
        self._widget.pushButtonFileOpen.clicked.connect(self.pushButton_clicked)
        self._widget.pushButtonFileOpen.setEnabled(False)

        # button (Connect)
        self._widget.pushButtonConnect.toggled.connect(self.pushButtonConnect_toggled)
        self._widget.pushButtonConnect.toggled.connect(self.publish)
        self._widget.pushButtonConnect.setEnabled(False)
        self._widget.pushButtonConnect_whole.toggled.connect(self.pushButtonConnect_whole_toggled)
        self._widget.pushButtonConnect_whole.toggled.connect(self.publish)
        self._widget.pushButtonConnect_whole.setEnabled(False)

        # double_editors (joint_group)
        self._widget.joint_group.setLayout(QVBoxLayout())

        # comboBox (Preset)
        self._widget.comboBoxPreset.activated[str].connect(self.setup_preset)

        # comboBox (AngleUnit)
        # self._widget.comboBoxAngleUnit.addItem("radian")
        # self._widget.comboBoxAngleUnit.addItem("degree")
        self._widget.comboBoxAngleUnit.activated[str].connect(self.publish)

        #rospy.Subscriber("/" + self._nameSpace + "/joint_states", JointState, self.callback, queue_size=1)
        #self._joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.callback, queue_size=1)
        self._joint_state_subscriber = rospy.Subscriber("/torobo/joint_state_server/joint_states", JointState, self.callback, queue_size=1)

        self._trajectory_command_publisher = {}
        for controller in self._controller_info.keys():
            if "gripper" in controller:
                #self._trajectory_command_publisher[controller] = rospy.Publisher("/" + self._nameSpace + "/" + controller + "/gripper_cmd/goal", GripperCommandActionGoal, queue_size=1)
                self._trajectory_command_publisher[controller] = actionlib.SimpleActionClient("/" + self._nameSpace + "/" + controller + "/gripper_cmd", GripperCommandAction)
                #self._trajectory_command_publisher[controller].wait_for_server()
            else:
                self._trajectory_command_publisher[controller] = rospy.Publisher("/" + self._nameSpace + "/" + controller + "/command", JointTrajectory, queue_size=1)

        # timer to update double_editor by callbackjoint_states) data
        self._timer = QTimer()
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(50)


    def pushButtonConnect_toggled(self, checked):
        if checked:
            self._widget.comboBoxControllerNamespace.setEnabled(False)
            self._widget.comboBoxControllerName.setEnabled(False)
            self._comboBoxTpList.setEnabled(False)
            self._widget.comboBoxPreset.setEnabled(False)
            self._widget.pushButtonSave.setEnabled(False)
            self._widget.pushButtonDelete.setEnabled(False)
            self._widget.pushButtonFileSave.setEnabled(False)
            self._widget.pushButtonFileOpen.setEnabled(False)
            self._widget.pushButtonConnect_whole.setEnabled(False)
            for de in self._double_editor_list:
                de.setWorkingColor(True)
        else:
            self._widget.comboBoxControllerNamespace.setEnabled(True)
            self._widget.comboBoxControllerName.setEnabled(True)
            self._comboBoxTpList.setEnabled(True)
            self._widget.comboBoxPreset.setEnabled(True)
            self._widget.pushButtonSave.setEnabled(True)
            self._widget.pushButtonDelete.setEnabled(True)
            self._widget.pushButtonFileSave.setEnabled(True)
            self._widget.pushButtonFileOpen.setEnabled(True)
            controller_name = str(self._widget.comboBoxControllerName.currentText())
            if "gripper" in controller_name:
                self._widget.pushButtonConnect_whole.setEnabled(False)
            else:
                self._widget.pushButtonConnect_whole.setEnabled(True)
            for de in self._double_editor_list:
                de.setWorkingColor(False)

    def pushButtonConnect_whole_toggled(self, checked):
        if checked:
            self._widget.comboBoxControllerNamespace.setEnabled(False)
            self._widget.comboBoxControllerName.setEnabled(False)
            self._comboBoxTpList.setEnabled(False)
            self._widget.comboBoxPreset.setEnabled(False)
            self._widget.pushButtonSave.setEnabled(False)
            self._widget.pushButtonDelete.setEnabled(False)
            self._widget.pushButtonFileSave.setEnabled(False)
            self._widget.pushButtonFileOpen.setEnabled(False)
            self._widget.pushButtonConnect.setEnabled(False)
            for de in self._double_editor_list:
                de.setWorkingColor(True)
        else:
            self._widget.comboBoxControllerNamespace.setEnabled(True)
            self._widget.comboBoxControllerName.setEnabled(True)
            self._comboBoxTpList.setEnabled(True)
            self._widget.comboBoxPreset.setEnabled(True)
            self._widget.pushButtonSave.setEnabled(True)
            self._widget.pushButtonDelete.setEnabled(True)
            self._widget.pushButtonFileSave.setEnabled(True)
            self._widget.pushButtonFileOpen.setEnabled(True)
            self._widget.pushButtonConnect.setEnabled(True)
            for de in self._double_editor_list:
                de.setWorkingColor(False)

    def destroy(self):
        for key in self._trajectory_command_publisher.keys():
            if hasattr(self._trajectory_command_publisher[key], 'unregister'):
                self._trajectory_command_publisher[key].unregister()
                self._trajectory_command_publisher[key] = None
        if self._display_planned_path_publisher is not None:
            self._display_planned_path_publisher.unregister()
            self._display_planned_path_publisher = None
        if self._joint_state_subscriber is not None:
            self._joint_state_subscriber.unregister()

    def get_controller_info(self, rosparam_name):
        info = {}
        data = repeat_get_param.get_param(rosparam_name)
        for l in data:
            fullname = l["name"]
            #if( "gripper" in fullname):
            #    continue
            name = fullname.replace(self._nameSpace.replace("/", ""), "").lstrip("/")
            dic = {}
            id = 0
            for j in l["joints"]:
                dic[j] = id
                id += 1
            info[name] = sorted(dic.items(), key=lambda x: x[1])
        info = OrderedDict(sorted(info.items(), key=lambda x: x[0]))
        return info

    def is_valid_rosparam_name(self, name):
        c = name[0]
        if not (c.isalpha() or c == '/' or c == '~'):
            return False
        for c in name:
            if not (c.isalnum() or c == '/' or c == '~'):
                return False
        return True 

    def pushButton_clicked(self):
        sender = self.sender()
        # Save button
        if sender == self._widget.pushButtonSave:
            if self._comboBoxTpList.currentIndex() < 0:
                return
            if self._comboBoxTpList.currentText() == "<New>":
                return
            tpName = str(self._comboBoxTpList.currentText())
            point = JointTrajectoryPoint()
            for de in self._double_editor_list:
                point.positions.append(de.targetValue())
            rosparam_name = "/" + self._nameSpace + "/" + str(self._widget.comboBoxControllerName.currentText())
            teaching_point_manager.RecordTeachingPointToRosParam(rosparam_name, tpName, point)

            myFont = QFont()
            myFont.setBold(False)
            self._widget.pushButtonSave.setFont(myFont)

        # Delete button
        elif sender == self._widget.pushButtonDelete:
            if self._comboBoxTpList.currentIndex() < 0:
                return
            if self._comboBoxTpList.currentText() == "<New>":
                return
            tpName = str(self._comboBoxTpList.currentText())
            rosparam_name = "/" + self._nameSpace + "/" + str(self._widget.comboBoxControllerName.currentText())
            teaching_point_manager.DeleteTeachingPointFromRosParam("/" + self._nameSpace + "/" + str(self._widget.comboBoxControllerName.currentText()), tpName)
            self._comboBoxTpList.removeItem(self._comboBoxTpList.currentIndex())

            myFont = QFont()
            myFont.setBold(False)
            self._widget.pushButtonSave.setFont(myFont)

        # FileSave button
        elif sender == self._widget.pushButtonFileSave:
            loadfile = QFileDialog.getSaveFileName(self._widget, "Save rosparam", "~/torobo_teaching_param.yaml", "Yaml (*.yaml)")
            fileName = loadfile[0]
            if fileName != "":
                param = {}
                for (name, dic) in self._controller_info.items():
                    if ("gripper" in name):
                        continue
                    param[name] = {}
                    tps = rospy.get_param("/" + self._nameSpace + "/" + name + "/teaching_points", None)
                    trajs = rospy.get_param("/" + self._nameSpace + "/" + name + "/teaching_trajectories", None)
                    if (tps != None):
                        param[name]['teaching_points'] = tps
                    if (trajs != None):
                        param[name]['teaching_trajectories'] = trajs

                with open(fileName, "w") as outfile:
                    yaml.dump(param, outfile, default_flow_style=False)
                rospy.loginfo("Saved rosparam to [%s]" % fileName)
            else:
                rospy.loginfo("Not saved")

        # FileOpen button
        elif sender == self._widget.pushButtonFileOpen:
            loadfile = QFileDialog.getOpenFileName(self._widget, "Please select loading rosparam yaml file", "~/", "Yaml (*.yaml)")
            fileName = loadfile[0]
            if(os.path.exists(fileName) == False):
                rospy.loginfo("[%s] is invalid rosparam file." % fileName)
                return
            #paramlist = rosparam.load_file(fileName, self._nameSpace)
            paramlist = rosparam.load_file(fileName)
            for params, ns in paramlist:
                rosparam.upload_params(ns, params)
            rospy.loginfo("Loaded rosparam from [%s]" % fileName)

            if self._widget.comboBoxControllerName.isEnabled():
                # update comboBoxTpList
                self._comboBoxTpList.clear()
                self._comboBoxTpList.addItem("<New>")
                names = teaching_point_manager.GetTeachingPointNamesFromRosParam("/" + self._nameSpace + "/" + str(self._widget.comboBoxControllerName.currentText()))
                if names is not None:
                    for name in names:
                        self._comboBoxTpList.addItem(name)
                self._comboBoxTpList.setCurrentIndex(-1)

                if len(self._double_editor_list) > 0:
                    self._widget.pushButtonConnect.setEnabled(True)
                    self._widget.pushButtonConnect_whole.setEnabled(True)
                    self._comboBoxTpList.setEnabled(True)
                else:
                    self._widget.pushButtonConnect.setEnabled(False)
                    self._widget.pushButtonConnect_whole.setEnabled(False)
                    self._comboBoxTpList.setEnabled(False)

    def target_value_changed(self):
        if self._current_tp == None:
            return
        i = 0
        for de in self._double_editor_list:
            if self._current_tp.positions[i] != de.targetValue():
                myFont = QFont()
                myFont.setBold(True)
                self._widget.pushButtonSave.setFont(myFont)
                return
            i += 1
        myFont = QFont()
        myFont.setBold(False)
        self._widget.pushButtonSave.setFont(myFont)

    def setup_joint_group(self):

        if len(self._JOINT_STATE_DATA_.name) == 0:
            rospy.loginfo("joint_state has not been ready")
            return

        if self._joint_limits is None:
            self._joint_limits = get_joint_limits()


        pre_tp_name = ""
        if self._comboBoxTpList.currentIndex() > 0:
            pre_tp_name = self._comboBoxTpList.currentText()

        # clear DoubleEditors
        def clearLayout(layout):
            while layout.count():
                child = layout.takeAt(0)
                if child.widget() is not None:
                    child.widget().deleteLater()
                elif child.layout() is not None:
                    clearLayout(child.layout())
        clearLayout(self._widget.joint_group.layout())

        controller_name = self._widget.comboBoxControllerName.currentText()

        if "gripper" in controller_name:
            current_unit = self._widget.comboBoxAngleUnit.currentText()
            self._widget.comboBoxAngleUnit.clear()
            self._widget.comboBoxAngleUnit.addItem("meter")
            self._widget.comboBoxAngleUnit.setCurrentIndex(0)

            self._widget.comboBoxPreset.clear()
            self._widget.comboBoxPreset.addItem("Current")
            self._widget.comboBoxPreset.addItem("Zero")
            self._widget.comboBoxPreset.setCurrentIndex(0)

        else:
            current_unit = self._widget.comboBoxAngleUnit.currentText()
            self._widget.comboBoxAngleUnit.clear()
            self._widget.comboBoxAngleUnit.addItem("radian")
            self._widget.comboBoxAngleUnit.addItem("degree")
            index = self._widget.comboBoxAngleUnit.findText(current_unit)
            if index < 0:
                index = 0
            self._widget.comboBoxAngleUnit.setCurrentIndex(index)

            self._widget.comboBoxPreset.clear()
            self._widget.comboBoxPreset.addItem("Current")
            self._widget.comboBoxPreset.addItem("Zero")
            self._widget.comboBoxPreset.addItem("Home")
            self._widget.comboBoxPreset.setCurrentIndex(0)


        # update comboBoxTpList
        self._comboBoxTpList.clear()
        if "gripper" in controller_name:
            pass
        else:
            self._comboBoxTpList.addItem("<New>")
            names = teaching_point_manager.GetTeachingPointNamesFromRosParam("/" + self._nameSpace + "/" + controller_name)
            if names is not None:
                for name in names:
                    self._comboBoxTpList.addItem(name)

        self._comboBoxTpList.setCurrentIndex(-1)
        tp = None
        if pre_tp_name == "<New>" or pre_tp_name == "":
            pass
        else:
            tp_index = self._comboBoxTpList.findText(pre_tp_name)
            if tp_index > 0:
                self._comboBoxTpList.setCurrentIndex(tp_index)
                tp = teaching_point_manager.GetTeachingPointFromRosParam("/" + self._nameSpace + "/" + controller_name, pre_tp_name)

        # add DoubleEditor
        self._double_editor_list = []
        i = 0
        for key, value in self._controller_info[controller_name]:
            current_position = self._JOINT_STATE_DATA_.position[self._JOINT_STATE_DATA_.name.index(key)]
            if tp != None:
                current_position = tp.positions[i]
            i += 1
            de = DoubleEditor(key, self._joint_limits[key]["min_position"], self._joint_limits[key]["max_position"], current_position, current_position)
            de.targetValueChanged.connect(self.publish)
            de.targetValueChanged.connect(self.target_value_changed)
            self._widget.joint_group.layout().addWidget(de)
            self._double_editor_list.append(de)

        # start move_group_commander service for forward kinematics
        move_group_name = str(controller_name[0:controller_name.find("_controller")])
        self._robot = moveit_commander.MoveGroupCommander(move_group_name)
        rospy.wait_for_service('compute_fk')
        try:
            self._moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)

        if len(self._double_editor_list) > 0:
            self._widget.pushButtonConnect.setEnabled(True)
            if "gripper" in controller_name:
                self._widget.pushButtonConnect_whole.setEnabled(False)
            else:
                self._widget.pushButtonConnect_whole.setEnabled(True)
            self._comboBoxTpList.setEnabled(True)
            self._widget.pushButtonSave.setEnabled(True)
            self._widget.pushButtonDelete.setEnabled(True)
            self._widget.pushButtonFileSave.setEnabled(True)
            self._widget.pushButtonFileOpen.setEnabled(True)
        else:
            self._widget.pushButtonConnect_whole.setEnabled(False)
            self._comboBoxTpList.setEnabled(False)
            self._widget.pushButtonSave.setEnabled(False)
            self._widget.pushButtonDelete.setEnabled(False)
            self._widget.pushButtonFileSave.setEnabled(False)
            self._widget.pushButtonFileOpen.setEnabled(False)

        myFont = QFont()
        myFont.setBold(False)
        self._widget.pushButtonSave.setFont(myFont)

        # publish once
        self.publish()

    def focus_released_tp(self):
        self._comboBoxTpList.setEditable(False)
        string = self._comboBoxTpList.currentText()
        if string == "<New>":
            self._comboBoxTpList.setCurrentIndex(-1)


    def edit_finish_tp(self):
        string = self._comboBoxTpList.currentText()
        # save new TP
        if string == "<New>":
            self._comboBoxTpList.setEditable(False)
            self._comboBoxTpList.setCurrentIndex(-1)
            self._current_tp = None
        elif not self.is_valid_rosparam_name(string):
            self._comboBoxTpList.setEditable(False)
            self._comboBoxTpList.removeItem(self._comboBoxTpList.currentIndex())
            self._comboBoxTpList.setCurrentIndex(-1)
            self._current_tp = None
        else:
            point = JointTrajectoryPoint()
            for de in self._double_editor_list:
                point.positions.append(de.targetValue())
            tpName = string
            rosparam_name = "/" + self._nameSpace + "/" + str(self._widget.comboBoxControllerName.currentText())
            teaching_point_manager.RecordTeachingPointToRosParam(rosparam_name, tpName, point)
            self._comboBoxTpList.setEditable(False)
            self._current_tp = point
        self.publish()

        myFont = QFont()
        myFont.setBold(False)
        self._widget.pushButtonSave.setFont(myFont)

    def setup_tp(self, string):
        print string
        self._current_tp = None
        if string == "<New>":
            self._comboBoxTpList.setEditable(True)
            self._comboBoxTpList.lineEdit().selectAll()
            return
        else:
            self._comboBoxTpList.setEditable(False)

        point = teaching_point_manager.GetTeachingPointFromRosParam("/" + self._nameSpace + "/" + str(self._widget.comboBoxControllerName.currentText()), string)
        print point
        if point != None:
            self._current_tp = point
            print point
            for i in range(len(self._double_editor_list)):
                self._double_editor_list[i].setTargetValue(point.positions[i])
                self._double_editor_list[i]._on_slider_value_changed()

        myFont = QFont()
        myFont.setBold(False)
        self._widget.pushButtonSave.setFont(myFont)

    def setup_preset(self, string):
        text = str(self._widget.comboBoxPreset.currentText())
        if text == "Zero":
            for de in self._double_editor_list:
                de.setTargetValue(0.0)
                de._on_slider_value_changed()
        elif text == "Home":
            rosparam_name = "/" + self._nameSpace + "/" + str(self._widget.comboBoxControllerName.currentText()) + "/home_position/"
            for de in self._double_editor_list:
                value = rospy.get_param(rosparam_name + de.text())
                de.setTargetValue(float(value))
                de._on_slider_value_changed()
        elif text == "Current":
            for de in self._double_editor_list:
                de.setTargetValue(de.currentValue())
                de._on_slider_value_changed()

    def update_xyzrpy(self):
        # Change angle unit
        controller = str(self._widget.comboBoxControllerName.currentText())
        if "gripper" in controller:
            self._widget.doubleSpinBox_roll.setDecimals(3)
            self._widget.doubleSpinBox_pitch.setDecimals(3)
            self._widget.doubleSpinBox_yaw.setDecimals(3)
            self._widget.doubleSpinBox_x.setValue(0.0)
            self._widget.doubleSpinBox_y.setValue(0.0)
            self._widget.doubleSpinBox_z.setValue(0.0)
            self._widget.doubleSpinBox_roll.setValue(0.0)
            self._widget.doubleSpinBox_pitch.setValue(0.0)
            self._widget.doubleSpinBox_yaw.setValue(0.0)
        else:
            if self._widget.comboBoxAngleUnit.currentText() == "radian":
                self._widget.doubleSpinBox_roll.setDecimals(3)
                self._widget.doubleSpinBox_pitch.setDecimals(3)
                self._widget.doubleSpinBox_yaw.setDecimals(3)
            else:
                self._widget.doubleSpinBox_roll.setDecimals(1)
                self._widget.doubleSpinBox_pitch.setDecimals(1)
                self._widget.doubleSpinBox_yaw.setDecimals(1)

            if True:
                # calculate by moveit
                state = RobotState()
                state.joint_state.name = []
                state.joint_state.position = []
                for de in self._double_editor_list:
                    state.joint_state.name.append(de.text())
                    state.joint_state.position.append(de.targetValue())
                fk_result = self._moveit_fk(Header(0, rospy.Time.now(), "/world"), [self._robot.get_end_effector_link()], state)
                pose = fk_result.pose_stamped[0].pose
                euler = tf.transformations.euler_from_quaternion(
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
                )
                self._widget.doubleSpinBox_x.setValue(pose.position.x)
                self._widget.doubleSpinBox_y.setValue(pose.position.y)
                self._widget.doubleSpinBox_z.setValue(pose.position.z)
                if self._widget.comboBoxAngleUnit.currentText() == "radian":
                    self._widget.doubleSpinBox_roll.setValue(euler[0])
                    self._widget.doubleSpinBox_pitch.setValue(euler[1])
                    self._widget.doubleSpinBox_yaw.setValue(euler[2])
                else:
                    self._widget.doubleSpinBox_roll.setValue(np.degrees(euler[0]))
                    self._widget.doubleSpinBox_pitch.setValue(np.degrees(euler[1]))
                    self._widget.doubleSpinBox_yaw.setValue(np.degrees(euler[2]))
            else:
                # calculate by tf2
                tfBuffer = tf2_ros.Buffer()
                listener = tf2_ros.TransformListener(tfBuffer)
                msg = tfBuffer.lookup_transform("world", self._robot.get_end_effector_link(), rospy.Time(), rospy.Duration(1.0)) # TODO: Get eef link name from torobo_control's config
                t = msg.transform
                q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
                euler = tf.transformations.euler_from_quaternion(q)
                self._widget.doubleSpinBox_x.setValue(t.translation.x)
                self._widget.doubleSpinBox_y.setValue(t.translation.y)
                self._widget.doubleSpinBox_z.setValue(t.translation.z)
                if self._widget.comboBoxAngleUnit.currentText() == "radian":
                    self._widget.doubleSpinBox_roll.setValue(euler[0])
                    self._widget.doubleSpinBox_pitch.setValue(euler[1])
                    self._widget.doubleSpinBox_yaw.setValue(euler[2])
                else:
                    self._widget.doubleSpinBox_roll.setValue(np.degrees(euler[0]))
                    self._widget.doubleSpinBox_pitch.setValue(np.degrees(euler[1]))
                    self._widget.doubleSpinBox_yaw.setValue(np.degrees(euler[2]))

    def publish(self):
        if len(self._double_editor_list) == 0:
            return

        current_controller = str(self._widget.comboBoxControllerName.currentText())

        unit = self._widget.comboBoxAngleUnit.currentText()
        for de in self._double_editor_list:
            if unit != de.unit():
                de.setUnit(unit)
        self.update_xyzrpy()

        transition_time = self._widget.doubleSpinBox_transitiontime.value()

        # create trajectory for Display Planned Path publisher
        if "gripper" not in current_controller:
            trajectory = JointTrajectory()
            trajectory.joint_names = []
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(5.0)
            point.positions = []
            if self._comboBoxTpList.currentIndex() < 0:
                string = ""
            else:
                string = self._comboBoxTpList.currentText()
            #if string == "" or string == "<New>" or ("gripper" in current_controller):
            if string == "" or string == "<New>":
                pass
            else:
                for controller in self._controller_info.keys():
                    if controller == str(self._widget.comboBoxControllerName.currentText()):
                        pass
                    elif "gripper" in controller:
                        pass
                    else:
                        tp_names = teaching_point_manager.GetTeachingPointNamesFromRosParam("/" + self._nameSpace + "/" + controller)
                        if string in tp_names:
                            tp = teaching_point_manager.GetTeachingPointFromRosParam("/" + self._nameSpace + "/" + controller, string)
                            if tp != None:
                                i = 0
                                for key, value in self._controller_info[controller]:
                                    trajectory.joint_names.append(key)
                                    point.positions.append(tp.positions[i])
                                    i += 1
            for de in self._double_editor_list:
                trajectory.joint_names.append(de.text())
                point.positions.append(de.targetValue())
            trajectory.points.append(point)
            rt = RobotTrajectory()
            rt.joint_trajectory = trajectory
            msg = DisplayTrajectory()
            msg.model_id = 'torobo'
            msg.trajectory.append(rt)
            msg.trajectory_start.joint_state = copy.deepcopy(self._JOINT_STATE_DATA_) # current value
            if self._display_planned_path_publisher is not None:
                self._display_planned_path_publisher.publish(msg)

        # create trajectory for single controller's trajectory command
        if self._widget.pushButtonConnect.isChecked():
            if "gripper" not in current_controller:
                trajectory = JointTrajectory()
                trajectory.joint_names = []
                point = JointTrajectoryPoint()
                point.time_from_start = rospy.Duration(transition_time)
                point.positions = []
                for de in self._double_editor_list:
                    trajectory.joint_names.append(de.text())
                    point.positions.append(de.targetValue())
                trajectory.points.append(point)
                if self._trajectory_command_publisher[current_controller] is not None:
                    self._trajectory_command_publisher[current_controller].publish(trajectory)
            else:
                goal = GripperCommandGoal()
                goal.command.position = self._double_editor_list[0].targetValue()
                goal.command.max_effort = 10.0
                self._trajectory_command_publisher[current_controller].send_goal(goal)
                self._trajectory_command_publisher[current_controller].wait_for_result(timeout=rospy.Duration(1.0))


        # create trajectory for whole controllers' trajectory command
        if self._widget.pushButtonConnect_whole.isChecked():
            # publish for current controller's trajectory command
            if self._comboBoxTpList.currentIndex() < 0:
                string = ""
            else:
                string = self._comboBoxTpList.currentText()
            if string == "" or string == "<New>":
                pass
            else:
                for controller in self._controller_info.keys():
                    if controller == str(self._widget.comboBoxControllerName.currentText()):
                        pass
                    elif "gripper" in controller:
                        pass
                    else:
                        tp_names = teaching_point_manager.GetTeachingPointNamesFromRosParam("/" + self._nameSpace + "/" + controller)
                        if string in tp_names:
                            tp = teaching_point_manager.GetTeachingPointFromRosParam("/" + self._nameSpace + "/" + controller, string)
                            if tp != None:
                                i = 0
                                trajectory = JointTrajectory()
                                trajectory.joint_names = []
                                point = JointTrajectoryPoint()
                                point.time_from_start = rospy.Duration(transition_time)
                                point.positions = []
                                for key, value in self._controller_info[controller]:
                                    trajectory.joint_names.append(key)
                                    point.positions.append(tp.positions[i])
                                    i += 1
                                trajectory.points.append(point)
                                self._trajectory_command_publisher[controller].publish(trajectory)
            # publish other controllers' trajectory command
            trajectory = JointTrajectory()
            trajectory.joint_names = []
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(transition_time)
            point.positions = []
            for de in self._double_editor_list:
                trajectory.joint_names.append(de.text())
                point.positions.append(de.targetValue())
                point.velocities.append(0.0)
                point.accelerations.append(0.0)
                point.effort.append(0.0)
            trajectory.points.append(point)
            if self._trajectory_command_publisher[current_controller] is not None:
                self._trajectory_command_publisher[current_controller].publish(trajectory)

    # callback for ROS's subscriber
    def callback(self, msg):
        self._JOINT_STATE_DATA_ = copy.deepcopy(msg)

    # callback for Timer
    def _on_timer(self):
        if self._double_editor_list is not None:
            for de in self._double_editor_list:
                position = self._JOINT_STATE_DATA_.position[self._JOINT_STATE_DATA_.name.index(de.text())]
                de.updateCurrentValue.emit(position)


# DoubleEditor (QFloatSlider/QFloatSpinBox with QFloatProgressBar)
class DoubleEditor(QWidget):
    
    targetValueChanged = Signal(float)
    updateCurrentValue = Signal(float)

    # each '*value' argument's unit must be "radian"
    def __init__(self, text, min_value, max_value, target_value, current_value):
        super(DoubleEditor, self).__init__()
        assert min_value < max_value, "DoubleEditor's minumum cannot be higher than maximum."

        self._text = text                   # Joint's name
        self._min_value = min_value         # Minimum value
        self._max_value = max_value         # Maximum value
        self._target_value = target_value   # Target value for slider
        self._current_value = current_value # Current value for progressbar
        self._unit = "radian"               # Unit for preview

        # Set gridLayout to use multiple overlay
        self.setLayout(QGridLayout())

        # label for joint name
        label_joint_name = QLabel(text)
        myFont = QFont()
        myFont.setBold(True)
        label_joint_name.setFont(myFont)
        self.layout().addWidget(label_joint_name, 0, 0)

        # progressbar for current value
        self.progressbar = QFloatProgressBar()
        self.progressbar.setRange(self._min_value, self._max_value)
        self.progressbar.setValue(self._current_value)
        self.layout().addWidget(self.progressbar, 0, 1)
        self.setWorkingColor(False)

        # label for minimum value
        self.label_min = QLabel(" {:.3f}".format(self._min_value))
        self.label_min.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.layout().addWidget(self.label_min, 0, 1)

        # label for maximum value
        self.label_max = QLabel("{:.3f} ".format(self._max_value))
        self.label_max.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.layout().addWidget(self.label_max, 0, 1)

        # slider for target value
        self.slider = QFloatSlider(Qt.Horizontal)
        self.slider.setRange(self._min_value, self._max_value)
        self.slider.setValue(self._target_value)
        self.layout().addWidget(self.slider, 0, 1)

        self.slider.setStyleSheet("""
        QSlider::groove:horizontal{
            border: 0px solid lightgrey;
            height: 0px;
            background: orange;
            margin: 0px 0;
            border-radius: 5px;
        }
        QSlider::handle:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f);
            border: 2px solid #5c5c5c;
            width: 6px;
            margin: -15px 0;
            border-radius: 5px;
        }
        QSlider::add-page:horizontal{
            border: 0px solid lightgrey;
            height: 0px;
            background: white;
            margin: 0px 0;
            border-radius: 5px;
        }
        """)

        # spinbox for target value
        self.spinbox = QFloatSpinBox()
        self.spinbox.setRange(self._min_value, self._max_value)
        self.spinbox.setValue(self._target_value)
        self.spinbox.setSingleStep(0.01)
        self.spinbox.setDecimals(3)
        self.spinbox.setFixedWidth(70)
        self.spinbox.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.layout().addWidget(self.spinbox, 0, 2)

        # connect between slider and spinbox
        self.slider.valueChanged.connect(self._on_slider_value_changed)
        self.spinbox.editingFinished.connect(self._on_spinbox_editing_finished)

        # Register ROS's subscriber
        self.updateCurrentValue.connect(self.setCurrentValue)
        #rospy.Subscriber("/joint_states", JointState, self.callback, queue_size=1)

    def setWorkingColor(self, flag):
        if flag:
            self.progressbar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #5c5c5c;
                border-radius: 5px;
                text-align: center;
                height: 2px;
                margin: 2px 0;
                background: white;
            }
            QProgressBar::chunk {
                /*background-color: #92D050;*/
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #92D050, stop:1 #70A030);
                width: 1px;
            }
            """)
        else:
            self.progressbar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #5c5c5c;
                border-radius: 5px;
                text-align: center;
                height: 2px;
                margin: 2px 0;
                background: white;
            }
            QProgressBar::chunk {
                /*background-color: #FFC000;*/
                /*background-color: #8f8f8f;*/
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #FFC000, stop:1 #CA9000);
                width: 1px;
            }
            """)

    def _on_slider_value_changed(self):
        value = self.slider.value()
        # Set radian's value
        if self._unit == "radian":
            self._target_value = value
        else:
            self._target_value = value * np.pi / 180.0
        # Update spinbox
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(value)
        self.spinbox.blockSignals(False)
        # Emit signal
        self.targetValueChanged.emit(value)

    def _on_spinbox_editing_finished(self):
        value = self.spinbox.value()
        # Set radian's value
        if self._unit == "radian":
            self._target_value = value
        else:
            self._target_value = value * np.pi / 180.0
        # Update slider
        self.slider.blockSignals(True)
        self.slider.setValue(value)
        self.slider.blockSignals(False)
        # Emit signal
        self.targetValueChanged.emit(value)

    # return joint name
    def text(self):
        return self._text

    # set target value. (unit must be "radian")        
    def setTargetValue(self, value):
        self._target_value = value
        self.spinbox.blockSignals(True)
        self.slider.blockSignals(True)
        if self._unit == "radian":
            self.spinbox.setValue(self._target_value)
            self.slider.setValue(self._target_value)
        else:
            self.spinbox.setValue(self._target_value * 180.0 / np.pi)
            self.slider.setValue(self._target_value * 180.0 / np.pi)
        self.spinbox.blockSignals(False)
        self.slider.blockSignals(False)

    # set current value. (unit must be "radian")        
    def setCurrentValue(self, value):
        self._current_value = value
        if self._unit == "radian":
            self.progressbar.setValue(self._current_value)
        else:
            self.progressbar.setValue(self._current_value * 180.0 / np.pi)

    # return target value. (unit is "radian")
    def targetValue(self):
        return self._target_value

    # return current value. (unit is "radian")
    def currentValue(self):
        return self._current_value

    # set unit (augument must be "radian" or "degree".)
    def setUnit(self, unit):
        self.slider.valueChanged.disconnect()
        self.spinbox.editingFinished.disconnect()

        self._unit = unit
        if self._unit == "radian":
            self.label_min.setText(" {:.3f}".format(self._min_value))
            self.label_max.setText("{:.3f} ".format(self._max_value))
            self.slider.setRange(self._min_value, self._max_value)
            self.spinbox.setRange(self._min_value, self._max_value)
            self.progressbar.setRange(self._min_value, self._max_value)
            self.progressbar.setDecimals(3)
            self.spinbox.setSingleStep(0.01)
            self.spinbox.setDecimals(3)
        elif self._unit == "degree":
            self.label_min.setText(" {:.1f}".format(self._min_value * 180.0 / np.pi))
            self.label_max.setText("{:.1f} ".format(self._max_value * 180.0 / np.pi))
            self.slider.setRange(self._min_value * 180 / np.pi, self._max_value * 180 / np.pi)
            self.spinbox.setRange(self._min_value * 180 / np.pi, self._max_value * 180 / np.pi)
            self.progressbar.setRange(self._min_value * 180 / np.pi, self._max_value * 180 / np.pi)
            self.progressbar.setDecimals(1)
            self.spinbox.setSingleStep(1)
            self.spinbox.setDecimals(1)
        else:
            self._unit = "radian" # meter
            self.label_min.setText(" {:.3f}".format(self._min_value))
            self.label_max.setText("{:.3f} ".format(self._max_value))
            self.slider.setRange(self._min_value, self._max_value)
            self.spinbox.setRange(self._min_value, self._max_value)
            self.progressbar.setRange(self._min_value, self._max_value)
            self.progressbar.setDecimals(3)
            self.spinbox.setSingleStep(0.01)
            self.spinbox.setDecimals(3)
        self.setTargetValue(self._target_value)
        self.setCurrentValue(self._current_value)

        self.slider.valueChanged.connect(self._on_slider_value_changed)
        self.spinbox.editingFinished.connect(self._on_spinbox_editing_finished)

    def unit(self):
        return self._unit


# Slider to deal with float value
class QFloatSlider(QSlider):

    def __init__(self, orientation):
        super(QFloatSlider, self).__init__(orientation)
        self._max_int = 10 ** 5
        super(QFloatSlider, self).setMinimum(0)
        super(QFloatSlider, self).setMaximum(self._max_int)
        self._min_value = -1000.0
        self._max_value = 1000.0

    @property
    def _value_range(self):
        return self._max_value - self._min_value

    def value(self):
        return float(super(QFloatSlider, self).value()) / self._max_int * self._value_range + self._min_value

    def setValue(self, value):
        super(QFloatSlider, self).setValue(int((value - self._min_value) / self._value_range * self._max_int))

    def setMinimum(self, value):
        self._min_value = value
        self.setValue(self.value())

    def setMaximum(self, value):
        self._max_value = value
        self.setValue(self.value())

    def setRange(self, min_value, max_value):
        assert min_value < max_value, "Slider's minimum cannot be higher than maximum."
        self.setMinimum(min_value)
        self.setMaximum(max_value)

    def minimum(self):
        return self._min_value

    def maximum(self):
        return self._max_value


# ProgressBar to deal with float value
class QFloatProgressBar(QProgressBar):

    def __init__(self):
        super(QFloatProgressBar, self).__init__()
        self._max_int = 10 ** 5
        super(QFloatProgressBar, self).setMinimum(0)
        super(QFloatProgressBar, self).setMaximum(self._max_int)
        self._min_value = 0.0
        self._max_value = 100.0
        self._format = '%.03f'
        self._float_value = 0.0
        self.valueChanged.connect(self.onValueChanged)

    def onValueChanged(self, value):
        self.setFormat(self._format % (self._float_value))

    @property
    def _value_range(self):
        return self._max_value - self._min_value

    def value(self):
        return float(super(QFloatProgressBar, self).value()) / self._max_int * self._value_range + self._min_value

    def setValue(self, value):
        self._float_value = value
        super(QFloatProgressBar, self).setValue(int(self._max_int * (value - self._min_value) / self._value_range))

    def setMinimum(self, value):
        self._min_value = value
        self.setValue(self.value())

    def setMaximum(self, value):
        self._max_value = value
        self.setValue(self.value())

    def setRange(self, min_value, max_value):
        assert min_value < max_value, "Progressbar's minimum cannot be higher than maximum."
        self.setMinimum(min_value)
        self.setMaximum(max_value)

    def minimum(self):
        return self._min_value

    def maximum(self):
        return self._max_value

    def setDecimals(self, decimal):
        self._format = '%.0' + str(decimal) + 'f'
        self.setFormat(self._format % (self._float_value))


# SpinBox to map mouse-left-click to signal of "editingFinished".
class QFloatSpinBox(QDoubleSpinBox):

    def __init__(self):
        super(QFloatSpinBox, self).__init__()
        self.mouse_left_pressed = False
        self.valueChanged.connect(self._on_value_changed)

    def mousePressEvent(self, e):
        super(QFloatSpinBox, self).mousePressEvent(e)
        if e.buttons() == Qt.LeftButton:
            self.mouse_left_pressed = True
            self.editingFinished.emit()
        else:
            self.mouse_left_pressed = False

    def mouseReleaseEvent(self, e):
        super(QFloatSpinBox, self).mouseReleaseEvent(e)
        self.mouse_left_pressed = False

    def _on_value_changed(self):
        if self.mouse_left_pressed:
            self.editingFinished.emit()


class EnhancedComboBox(QComboBox):
    """Enhanced version of QtGui.QComboBox which has an editingFinished-signal like QLineEdit."""
    editingFinished = pyqtSignal()
    focusReleased = pyqtSignal()
    _popup = None # The lineedit's contextmenu while it is shown
  
    def __init__(self,parent=None):
        super(self.__class__, self).__init__(parent)
        self.setEditable(True)
        self.lineEdit().installEventFilter(self)
  
    def focusOutEvent(self,focusEvent):
        if not self.view().isVisible() and self._popup is None:
            self.focusReleased.emit()
        QComboBox.focusOutEvent(self,focusEvent)
  
    def keyPressEvent(self,keyEvent):
        super(self.__class__, self).keyPressEvent(keyEvent)
        if keyEvent.key() == Qt.Key_Return or keyEvent.key() == Qt.Key_Enter:
            if not self.view().isVisible() and self._popup is None:
                self.editingFinished.emit()
  
    def eventFilter(self,object,event):
        if event.type() == QEvent.ContextMenu and object == self.lineEdit():
            self._popup = self.lineEdit().createStandardContextMenu()
            self._popup.exec_(event.globalPos())
            self._popup = None
            return True
        return False # don't stop the event
