#!/usr/bin/env python
## -*- coding: utf-8 -*-
import os
import rospy
import rosparam
import rospkg
import cv2
import threading
import csv

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

from cv_bridge import CvBridge, CvBridgeError
import numpy
import yaml

import torobo_driver.torobo_easy_command
from torobo_msgs.msg import ToroboJointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from torobo_driver import servo_on_client
from torobo_driver import servo_off_client
from torobo_driver import error_reset_client
from torobo_driver import set_control_mode_client
from torobo_motion_manager import move_home_position_client
from torobo_motion_manager import teaching_point_manager
from torobo_motion_manager import teaching_trajectory_manager
from torobo_control import get_torobo_joint_state_client

from torobo_common import repeat_get_param

#Take photo, Add by Saito
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
lock = threading.Lock()
class ImageBuffer(object):
    def __init__(self):
        self._image = None

    def is_empty(self):
        # Need Lock
        with lock:
            return self._image is None

    def push_image(self, img):
        # Need Lock
        with lock:
            self._image = img

    def pop_image(self):
        # Need Lock
        with lock:
            img = self._image
            self._image = None
            return img

class ToroboWholeBodyManager(Plugin):
    def __init__(self, context):
        super(self.__class__, self).__init__(context)
        self.setObjectName('ToroboWholeBodyManager')
        self._touch = None
        
        #Add by Saito
        self.touch_file=None
        self.touch_writer=None

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
        ui_file = os.path.join(rospkg.RosPack().get_path('torobo_gui'), 'resource', 'torobo_whole_body_manager.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ToroboWholeBodyManagerUi')

        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.serial_number = context.serial_number()
        ns = rospy.get_namespace()
        self.create(ns, 200)

	#Add by Saito
        rospy.Subscriber("/image_raw", Image, self.cb_image)
        rospy.Subscriber("/touchence/sensor_data",Float32MultiArray,self.TouchSensorCallback)
        self.steps=0
        self.bridge=CvBridge()
        self._imageBuffer = ImageBuffer()
        self.interval_para=1  #change here! 
        #If you want to do direct teaching and replay it, the parameter shoud be 1.
        #If you want to collect data, it should be 4 (then, the interval of data driven changes to 1/4 of you command in the GUI).
        
        #Add by Shimizu
        DATA_DIR = "/home/assimilation/TAKUMI_SHIMIZU/wiping_ws/src/wiping/data/"
        self._img_dir = DATA_DIR + "img/"
        self._mode = "teaching"


    #Add by Saito
    def cb_image(self, image):
        try:
            self._imageBuffer.push_image(self.bridge.imgmsg_to_cv2(image, "bgr8"))           
        except:
            traceback.print_exc()

    #Add by Saito
    def TouchSensorCallback(self, data):
        self._touch=data.data

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
    
    def create(self, nameSpace, updateTimeInterval):
        if (nameSpace == "/"):
            nameSpace = "torobo/"
        if (nameSpace[-1] != "/"):
            nameSpace += "/"
        self.nameSpace = nameSpace
        self.controllerListNameSpace = "/move_group/controller_list"

        self.controllerDict = self.GetControllerDict(nameSpace + self.controllerListNameSpace)
        self.toroboJointStateJointNames = {}
        self.goHomeActionServerConnection = False
        self.moveTeachingPointActionServerConnection = False
        self.moveTeachingTrajectoryActionServerConnection = False

        self.CreateServiceClient(self.controllerDict)

        self.initMainForm()

        self.updateTimer = QTimer(self)
        self.updateTimer.timeout.connect(self.update)
        self.updateTimer.start(updateTimeInterval)

        self.recordTimer = QTimer(self)
        self.recordTimer.timeout.connect(self.TrajectoryRecordCallback)
        self.recordPointsNum = 0

    def destroy(self):
        self.updateTimer.stop()
        self.recordTimer.stop()

    def GetControllerDict(self, controllerListNameSpace):
        dic = {}
        controllerNames = self.GetControllerNames(controllerListNameSpace)
        for name in controllerNames:
            jointNameIdDict = self.GetJointNameIdDict(controllerListNameSpace, name)
            dic[name] = jointNameIdDict
        return dic

    def GetJointNameIdDict(self, controllerListNameSpace, controllerName):
        dic = {}
        controller_list = repeat_get_param.get_param(controllerListNameSpace)
        if (controller_list is None):
            return dic
        id = 0
        for l in controller_list:
            if controllerName in l["name"]:
                for j in l["joints"]:
                    dic[j] = id
                    id += 1
        return dic

    def GetControllerNames(self, controllerListNameSpace):
        names = []
        controller_list = repeat_get_param.get_param(controllerListNameSpace)
        if (controller_list is None):
            return names
        tempns = self.nameSpace.replace("/", "")
        for l in controller_list:
            fullname = l["name"]
            name = fullname.replace(tempns, "")
            name = name.lstrip("/")
            names.append(name)
        return names

    def CreateServiceClient(self, controllerDict):
        self.servoOffClient = {}
        self.servoOnClient = {}
        self.errorResetClient = {}
        self.setControlModeClient = {}
        for(k, v) in controllerDict.items():
            self.servoOffClient[k] = servo_off_client.ServoOffClient(self.nameSpace + k)
            self.servoOnClient[k] = servo_on_client.ServoOnClient(self.nameSpace + k)
            self.errorResetClient[k] = error_reset_client.ErrorResetClient(self.nameSpace + k)
            self.setControlModeClient[k] = set_control_mode_client.SetControlModeClient(self.nameSpace + k)
    
    def initMainForm(self):
        # button
        self._widget.buttonAllServoOn.clicked.connect(self.button_clicked)
        self._widget.buttonAllServoOff.clicked.connect(self.button_clicked)
        self._widget.buttonAllErrorReset.clicked.connect(self.button_clicked)
        self._widget.buttonTeachingMode.clicked.connect(self.button_clicked)
        self._widget.buttonMovingMode.clicked.connect(self.button_clicked)
        self._widget.buttonGoHome.clicked.connect(self.button_clicked)
        self._widget.buttonGetTp.clicked.connect(self.button_clicked)
        self._widget.buttonMoveTp.clicked.connect(self.button_clicked)
        self._widget.buttonTrajRecord.clicked.connect(self.button_clicked)
        self._widget.buttonTrajRun.clicked.connect(self.button_clicked)
        self._widget.buttonSaveRosParam.clicked.connect(self.button_clicked)
        self._widget.buttonLoadRosParam.clicked.connect(self.button_clicked)

        #Add by shimizu
        self._widget.buttonTeach.clicked.connect(self.button_clicked)
        self._widget.buttonReplay.clicked.connect(self.button_clicked)
        self._widget.buttonGoStart.clicked.connect(self.button_clicked)


    def update(self):
        for(k, jointNames) in self.controllerDict.items():
            v = get_torobo_joint_state_client.call_service(self.nameSpace, k)
            if v is None:
                continue
            errorJoint = []
            servoState = "Stop"
            servoStateColor = QColor(Qt.black)
            mode = "Unknown"
            modeColor = QColor(Qt.black)
            for i in range(len(v.name)):
                systemMode = v.systemMode[i]
                if systemMode == eSystemMode.ERROR:
                    servoState = "Error"
                    servoStateColor = QColor(Qt.red)
                    errorJoint.append(i+1)
                elif errorJoint == [] and systemMode == eSystemMode.RUN:
                    servoState = "Run"
                    servoStateColor = QColor(Qt.blue)
            if servoState == "Error":
                for i,j in enumerate(errorJoint):
                    if i == 0:
                        servoState += " "
                    else:
                        servoState += "/"
                    servoState += str(j)

            controlMode = v.ctrlMode[0]
            if controlMode == eHighCtrlMode.TRAJ:
                mode = "Moving"
                modeColor = QColor(Qt.red)
            elif controlMode == eHighCtrlMode.EXTERNAL_FORCE_FOLLOWING:
                mode = "Teaching"
                modeColor = QColor(Qt.blue)

            if "left_arm" in k:
                self._widget.labelLArmMode.setText(mode)
                self._widget.labelLArmServoState.setText(servoState)
                pal = self._widget.labelLArmServoState.palette()
                pal.setColor(QPalette.Foreground,servoStateColor)
                self._widget.labelLArmServoState.setPalette(pal)
                pal = self._widget.labelLArmMode.palette()
                pal.setColor(QPalette.Foreground,modeColor)
                self._widget.labelLArmMode.setPalette(pal)
            elif "right_arm" in k:
                self._widget.labelRArmMode.setText(mode)
                self._widget.labelRArmServoState.setText(servoState)
                pal = self._widget.labelRArmServoState.palette()
                pal.setColor(QPalette.Foreground,servoStateColor)
                self._widget.labelRAReplayrmServoState.setPalette(pal)
                pal = self._widget.labelRArmMode.palette()
                pal.setColor(QPalette.Foreground,modeColor)
                self._widget.labelRArmMode.setPalette(pal)
            elif "torso_head" in k:
                self._widget.labelTorsoHeadMode.setText(mode)
                self._widget.labelTorsoHeadServoState.setText(servoState)
                pal = self._widget.labelTorsoHeadServoState.palette()
                pal.setColor(QPalette.Foreground,servoStateColor)
                self._widget.labelTorsoHeadServoState.setPalette(pal)
                pal = self._widget.labelTorsoHeadMode.palette()
                pal.setColor(QPalette.Foreground,modeColor)
                self._widget.labelTorsoHeadMode.setPalette(pal)

    def button_clicked(self):
        sender = self.sender()
        tpNum = int(self._widget.spinBoxTpNumber.text())
        tpName = "tp" + str(tpNum)
        trajName = unicode(self._widget.lineEditTrajName.text())
        transitionTime = float(self._widget.doubleSpinBoxTransitionTime.text())
        origin_recordInterval = float(self._widget.doubleSpinBoxRecordInterval.text())
        recordInterval = origin_recordInterval / self.interval_para # SAITO 

        def CallClassServiceAllController(classDict, sleepTime=0.0):
            for (name, c) in classDict.items():
                c.call_service("all")
                rospy.sleep(sleepTime)

        def CallClassServiceWithArgAllController(classDict, arg, sleepTime=0.0):
            for (name, c) in classDict.items():
                c.call_service("all", arg)
                rospy.sleep(sleepTime)
        if sender == self._widget.buttonAllServoOn:
            CallClassServiceAllController(self.servoOnClient, sleepTime=0.2)
        elif sender == self._widget.buttonAllServoOff:
            CallClassServiceAllController(self.servoOffClient)
        elif sender == self._widget.buttonAllErrorReset:
            CallClassServiceAllController(self.errorResetClient)
        elif sender == self._widget.buttonTeachingMode:
            CallClassServiceWithArgAllController(self.setControlModeClient, "external_force_following")
        elif sender == self._widget.buttonMovingMode:
            CallClassServiceWithArgAllController(self.setControlModeClient, "position")
        elif sender == self._widget.buttonGetTp:
            self.GoTp(tpName)
            self._widget.spinBoxTpNumber.setValue(tpNum+1)
        elif sender == self._widget.buttonMoveTp:
            self.MoveTeachingPoint(tpName, transitionTime)
            self._widget.spinBoxTpNumber.setValue(tpNum+1)
        elif sender == self._widget.buttonGoHome:
            self.GoHome(transitionTime)
        elif sender == self._widget.buttonTrajRecord:
            if self._widget.buttonTrajRecord.text() == "Record\nStart":
                self._widget.buttonTrajRecord.setText("Record\nStop")
                self._widget.buttonTrajRun.setEnabled(False)
                self._widget.buttonTeachingMode.click()
                self.StartTrajectoryRecord(recordInterval)
            else:
                self.StopTrajectoryRecord()
                self.RecordTrajectory(trajName)
                self._widget.buttonTrajRecord.setText("Record\nStart")
                self._widget.buttonTrajRun.setEnabled(True)
        elif sender == self._widget.buttonTrajRun:
            self.MoveTeachingTrajectory(trajName)
            self._widget.buttonTrajRecord.setText("Record\nStop")
            self.StartTrajectoryRecord_for_retry(recordInterval) #Add by Saito
        elif sender == self._widget.buttonSaveRosParam:
            self.SaveRosParam()
        elif sender == self._widget.buttonLoadRosParam:
            self.LoadRosParam()


        #Add by Shimizu
        elif sender == self._widget.buttonGoStart:
            # self.
            STARTPOSI_TP = "tp" + str(99)
            if self._widget.buttonGoStart.text() == "first start posi":
                self._widget.buttonLoadRosParam.click()
                self._widget.buttonMovingMode.click()
                self.MoveTeachingTrajectory(trajName)
                self._widget.buttonGoStart.setText("finish move")
            elif self._widget.buttonGoStart.text() == "finish move":
                #[TODO] I want to delete this. If I can extract the last position of start posi, 
                # then I can delete this process.
                self.GoTp(STARTPOSI_TP)
                self._widget.buttonGoStart.setText("go start posi")
            elif self._widget.buttonGoStart.text() == "go start posi":
                self._widget.buttonMovingMode.click()
                self.MoveTeachingPoint(STARTPOSI_TP, transitionTime)
                
        elif sender ==self._widget.buttonTeach:
            if self._widget.buttonTeach.text() == "teach start":
                self._widget.buttonTeach.setText("teach stop")
                self._widget.buttonTeachingMode.click()
                self.restart_record(origin_recordInterval)
                self._mode = "teaching"
            else:
                self._widget.buttonTeach.setText("teach start")
                #[TODO] 
                # back to start posi
                # teach finish
                self.recordTimer.stop()
                self.RecordTrajectory(trajName)
        elif sender == self._widget.buttonReplay:
            self._widget.buttonMovingMode.click()
            self.MoveTeachingTrajectory(trajName)
            # self.StartTrajectoryRecord_for_retry(recordInterval) #Add by Saito
            # retrace like traj run

    def GoTp(self,tpName):
        for (name, dic) in self.controllerDict.items():
            if ("gripper" in name):
                continue
            point = self.GetJointTrajectoryPoint(name)
            if point is None:
                continue
            teaching_point_manager.RecordTeachingPointToRosParam(self.nameSpace + name, tpName, point)
    def GoHome(self, transitionTime):
        if(self.goHomeActionServerConnection == False):
            # pre-connect to action server for avoiding delay
            ret = True
            for (name, dic) in self.controllerDict.items():
                if ("gripper" in name):
                    continue
                ret &= move_home_position_client.connect_server(self.nameSpace + name, timeout=1.0)
            if(ret):
                self.goHomeActionServerConnection = True

        self._widget.buttonMovingMode.click()
        for (name, dic) in self.controllerDict.items():
            if ("gripper" in name):
                continue
            move_home_position_client.call_action(self.nameSpace + name, transitionTime, timeout=0.5)

    def MoveTeachingPoint(self, tpName, transitionTime):
        if(self.moveTeachingPointActionServerConnection == False):
            # pre-connect to action server for avoiding delay
            ret = True
            for (name, dic) in self.controllerDict.items():
                if ("gripper" in name):
                    continue
                ret &= teaching_point_manager.connect_server(self.nameSpace + name, timeout=1.0)
            if(ret):
                self.moveTeachingPointActionServerConnection = True
        for (name, dic) in self.controllerDict.items():
            if ("gripper" in name):
                continue
            teaching_point_manager.MoveToTeachingPoint(self.nameSpace + name, tpName, transitionTime, timeout=0.5)

    #Add by Saito
    def MoveTeachingTrajectory(self, trajName):
        if(self.moveTeachingTrajectoryActionServerConnection == False):
            # pre-connect to action server for avoiding delay
            ret = True
            for (name, dic) in self.controllerDict.items():
                if ("gripper" in name):
                    continue
                ret &= teaching_trajectory_manager.connect_server(self.nameSpace + name, timeout=1.0)
            if(ret):
                self.moveTeachingTrajectoryActionServerConnection = True
        for (name, dic) in self.controllerDict.items():
            if ("gripper" in name):
                continue
            teaching_trajectory_manager.MoveToTeachingTrajectory(self.nameSpace + name, trajName, timeout=0.5)
        print "retry"

    def restart_record(self,recordInterval):
        
        self.recordPoints = {}
        for (name, dic) in self.controllerDict.items():
            if ("gripper" in name):
                continue
            self.recordPoints[name] = []
        self.recordPointsNum = 0
        self.recordIntervalMS = recordInterval * 1000.0
        self.recordTimer.start(self.recordIntervalMS)

    def StartTrajectoryRecord(self, recordInterval, startTimer=True):
        print "Record Start"
        self.restart_record(recordInterval)
        files_in_path=os.listdir("/home/assimilation/touchsensor/")
        number_of_files=len(files_in_path)
        self.touch_file=open("/home/assimilation/touchsensor/"+str(number_of_files+1)+".csv",'w')
        self.touch_writer=csv.writer(self.touch_file,lineterminator='\n')

    def StartTrajectoryRecord_for_retry(self, recordInterval, startTimer=True):
        print "Figure Record Start"
        self.restart_record(recordInterval)
        files_in_path=os.listdir("/home/assimilation/touchsensor/")
        number_of_files=len(files_in_path)
        self.touch_file=open("/home/assimilation/touchsensor/"+str(number_of_files+1)+".csv",'w')
        self.touch_writer=csv.writer(self.touch_file,lineterminator='\n')

    def StopTrajectoryRecord(self):
        print "Record Stop"
        self.recordTimer.stop()
        #Add by Saito
        self.touch_file.close()
    
    def RecordTrajectory(self, trajName):
        for (name, dic) in self.controllerDict.items():
            if ("gripper" in name):
                continue
            traj = JointTrajectory()
            traj.joint_names = self.toroboJointStateJointNames[name]
            traj.points = self.recordPoints[name]
            teaching_trajectory_manager.RecordTeachingTrajectoryToRosParam(self.nameSpace + name, trajName, traj)

    def IncrementRecordTime(self):
        self.recordTime = numpy.round(self.recordIntervalMS * self.recordPointsNum / 1000.0, 2)
    
    def GetJointTrajectoryPoint(self, controllerName):
        toroboJointState = get_torobo_joint_state_client.call_service(self.nameSpace, controllerName)
        if(toroboJointState is None):
            return None
        self.toroboJointStateJointNames[controllerName] = toroboJointState.name
        point = JointTrajectoryPoint()
        point.positions = toroboJointState.position
        point.velocities = toroboJointState.velocity
        point.accelerations = toroboJointState.acceleration
        point.effort = toroboJointState.effort
        #touch sensor Add by Saito
        #print self._touch
        if self._mode != "teaching":
            self.touch_writer.writerow(self._touch)

	#camera Add by Saito
    	if not self._imageBuffer.is_empty():
                if self.steps%self.interval_para==0:
		    cv_img = self._imageBuffer.pop_image()
            #Change by Shimizu
		    cv2.imwrite(self._img_dir + "{:0>4d}".format(self.steps/self.interval_para) + ".jpg", cv_img)
		self.steps=self.steps+1

        return point

    def RecordOnePoint(self):
        self.IncrementRecordTime()
        for (name, dic) in self.controllerDict.items():
            if ("gripper" in name):
                continue
            point = self.GetJointTrajectoryPoint(name)
            if(point is None):
                continue
            point.time_from_start = rospy.Duration.from_sec(self.recordTime)
            self.recordPoints[name].append(point)
        self.recordPointsNum += 1

    def TrajectoryRecordCallback(self):
        self.RecordOnePoint()
        self._widget.lcdNumberRecordTime.display(str(self.recordTime))
        self._widget.lcdNumberRecordedPoints.display(str(self.recordPointsNum))

    def SaveRosParam(self):
        loadfile = QFileDialog.getSaveFileName(self._widget, "Save rosparam", "~/torobo_teaching_param.yaml", "Yaml (*.yaml)")
        fileName = loadfile[0]
        if fileName != "":
            param = {}
            for (name, dic) in self.controllerDict.items():
                if ("gripper" in name):
                    continue
                param[name] = {}
                tps = rospy.get_param(self.nameSpace + name + "/teaching_points", None)
                trajs = rospy.get_param(self.nameSpace + name + "/teaching_trajectories", None)
                if (tps != None):
                    param[name]['teaching_points'] = tps
                if (trajs != None):
                    param[name]['teaching_trajectories'] = trajs

            with open(fileName, "w") as outfile:
                yaml.dump(param, outfile, default_flow_style=False)
            rospy.loginfo("Saved rosparam to [%s]" % fileName)
        else:
            rospy.loginfo("Not saved")

    def LoadRosParamFile(self, fileName):
        if(os.path.exists(fileName) == False):
            rospy.loginfo("[%s] is invalid rosparam file." % fileName)
            return
        paramlist = rosparam.load_file(fileName, self.nameSpace)
        for params, ns in paramlist:
            rosparam.upload_params(ns, params)
        rospy.loginfo("Loaded rosparam from [%s]" % fileName)

    def LoadRosParam(self):
        loadfile = QFileDialog.getOpenFileName(self._widget, "Please select loading rosparam yaml file", "~/", "Yaml (*.yaml)")
        fileName = loadfile[0]
        self.LoadRosParamFile(fileName)
        

    def OpenSaveFileDialog(self):
        filename = QFileDialog.getSaveFileName(self._widget, "Save rosparam", "~/torobo_teaching_param.yaml", "Yaml (*.yaml)")
        return filename

class eSystemMode:
    STOP = 0
    READY = 1
    RUN = 2
    ERROR = 3

class eHighCtrlMode:
    TRAJ = 0
    EXTERNAL_FORCE_FOLLOWING = 5

