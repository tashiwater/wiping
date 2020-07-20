#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
import cv2
import csv
import threading
from chainer import Variable
from chainer import functions
from chainer import serializers

import sys,os
sys.path.append(os.path.abspath(os.path.dirname(__file__))+'/Network/')
from Utils.dataUtils import *
from Utils.argUtils import *
from Utils.logUtils import *
from CNNmodel.DCNNAE import DCNNAE
from MTRNN.MTRNN import MTRNN
from MTRNN.MTRNN import InitialState
import traceback,sys,os
from cv_bridge import CvBridge, CvBridgeError
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
lock = threading.Lock()

MotionStep=108
TIME_INTERVAL=1
IO=22
Cf=160
Cs=13
input_param=0.8 #0.0=>close 1.0=>open

model = DCNNAE(inchannel=3)
serializers.load_npz("/home/assimilation/catkin_ws/src/torobo_robot/torobo_demo/scripts/arm/Network/CAE/20200203/snap_iter_1000.cnnmodel", model)
MT_model = MTRNN([IO,Cf,Cs], [2,5,50], 1)
serializers.load_npz("/home/assimilation/catkin_ws/src/torobo_robot/torobo_demo/scripts/arm/Network/MTRNN_data/20200206_mot_img/iter_20000.rnnmodel",MT_model)

class IMAGEBUFFER(object):
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

touch_file=open("/home/assimilation/cameratest/test2/touch.csv",'w')
touch_writer=csv.writer(touch_file,lineterminator='\n')
angle_file=open("/home/assimilation/cameratest/test2/angle.csv",'w')
angle_writer=csv.writer(angle_file,lineterminator='\n')

def normalization(data, indataRange, outdataRange):
    data=(data - indataRange[0])/(indataRange[1]-indataRange[0])
    data=data*(outdataRange[1] - outdataRange[0])+outdataRange[0]
    return data

i=0
cv_img=None
position=None
effort=None
tactile=None
def cb_image(image):
    global i, cv_img
    ImageBuffer=IMAGEBUFFER()
    bridge=CvBridge()
    try:
        ImageBuffer.push_image(bridge.imgmsg_to_cv2(image, "bgr8"))
        cv_img = ImageBuffer.pop_image() 
    except:
        traceback.print_exc()

def TouchSensorCallback(data):
    global tactile
    tactile = data.data
    touch_writer.writerow(tactile)

def ToroboCallback(data):
    global effort, position 
    position = data.position
    effort = data.effort

def main():
    global i, cv_img, effort, position, tactile
    rospy.Subscriber("/image_raw", Image, cb_image)
    rospy.Subscriber("/touchence/sensor_data",Float32MultiArray,TouchSensorCallback)
    rospy.Subscriber("/torobo/joint_state_server/joint_states",JointState,ToroboCallback)

    ACTION_SERVICE_NAME = '/torobo/arm_controller/follow_joint_trajectory'
    JOINT_NAMES = ['arm/joint_' + str(i) for i in range(1, 8)] # from joint_1 to joint_8

    try:
        # Initializes a rospy node.
        rospy.init_node('toroboarm_follow_trajectory_node')
        cv2.imwrite("/home/assimilation/cameratest/test2/0000.jpg", cv_img)
        for i in range(MotionStep-1):
            #CAE
            flist=glob.glob("/home/assimilation/cameratest/test2/%04d.jpg"%(i))
            img=Data_transform(flist, checkFileType(flist), 96, 128,3, [0,255])
            imgs_all=[]
            inbatch = np.ndarray((1, 3, 96, 128), dtype=np.float32)
            get_batch(1, 0, inbatch, img)
            x = Variable(np.asarray(inbatch)) 
            y = model(x, train=False) 
            imgs_all.append(y.data)

            #Save image feature
            imgs_all=np.array(imgs_all)
            imgs_all=imgs_all.reshape((img.shape[0]/1,1,imgs_all.shape[2]))
            
            #normalization
            mot1=normalization(float(position[0]),[-1.309,4.451],[-0.9,0.9])
            mot2=normalization(float(position[1]),[-2.094,0.140],[-0.9,0.9])
            mot3=normalization(float(position[2]),[-2.880,2.880],[-0.9,0.9])
            mot4=normalization(float(position[3]),[-0.524,2.269],[-0.9,0.9])
            mot5=normalization(float(position[4]),[-2.880,2.880],[-0.9,0.9])
            mot6=normalization(float(position[5]),[-1.396,1.396],[-0.9,0.9])
            mot7=normalization(float(position[6]),[-1.396,0.087],[-0.9,0.9])
            """
            eff1=normalization(float(effort[0]),[-2,40],[-0.9,0.9])
            eff2=normalization(float(effort[1]),[-40,15],[-0.9,0.9])
            eff3=normalization(float(effort[2]),[-5,15],[-0.9,0.9])
            eff4=normalization(float(effort[3]),[-10,15],[-0.9,0.9])
            eff5=normalization(float(effort[4]),[-5,5],[-0.9,0.9])
            eff6=normalization(float(effort[5]),[-5,5],[-0.9,0.9])
            eff7=normalization(float(effort[6]),[-5,5],[-0.9,0.9])
            """
            tac1=normalization(float(tactile[0]),[0,1],[-0.9,0.9])
            tac2=normalization(float(tactile[1]),[0,1],[-0.9,0.9])
            tac3=normalization(float(tactile[2]),[0,1],[-0.9,0.9])
            tac4=normalization(float(tactile[3]),[0,1],[-0.9,0.9])
            tac5=normalization(float(tactile[4]),[0,1],[-0.9,0.9])
            tac6=normalization(float(tactile[5]),[0,1],[-0.9,0.9])
            tac7=normalization(float(tactile[6]),[0,1],[-0.9,0.9])
            tac8=normalization(float(tactile[7]),[0,1],[-0.9,0.9])
            tac9=normalization(float(tactile[8]),[0,1],[-0.9,0.9])
            tac10=normalization(float(tactile[9]),[0,1],[-0.9,0.9])
            tac11=normalization(float(tactile[10]),[0,1],[-0.9,0.9])
            tac12=normalization(float(tactile[11]),[0,1],[-0.9,0.9])

            feature=normalization(imgs_all,[0,1],[-0.9,0.9])
            indata=[mot1,mot2,mot3,mot4,mot5,mot6,mot7]#,tac1,tac2,tac3,tac4,tac5,tac6,tac7,tac8,tac9,tac10,tac11,tac12]
            indata=np.append(indata,feature)
            #indata=np.append([],indata)
            indata=indata.reshape(1,IO)
            indata=np.array(indata,dtype=np.float32)

            if i<10:
                input_param_img=0.1
                input_param_tactile=0.1
            else:
                input_param_img=0.2
                input_param_tactile=0.2

            #MTRNN
            if i==0:
                u_io = Variable(np.zeros((1,IO)).astype(np.float32))
                u_cf = Variable(np.zeros((1,Cf)).astype(np.float32))
                u_cs = Variable(np.zeros((1,Cs)).astype(np.float32))
                cf = functions.tanh(u_cf)
                cs = functions.tanh(u_cs)
                x = Variable(indata)
            else:
		#tactile_x= input_param_tactile*indata[0,7:19].reshape(1,12)+(1.0-input_param_tactile)*io.data[0,7:19].reshape(1,12)
		img_x= input_param_img*indata[0,7:].reshape(1,15)+(1.0-input_param_img)*io.data[0,7:].reshape(1,15)
                mot_x = io.data[0,:7].reshape(1,7)
		#x=np.append(mot_x,tactile_x,axis=1)
		x=np.append(mot_x,img_x,axis=1)
		x=Variable(x)
            u_io, u_cf, u_cs, io, cf, cs = MT_model.forward_partial(u_io, u_cf, u_cs, x, cf, cs)
            #Cs_save.write(str(cs.data)+"\n")
            #Cs_data=np.append(Cs_data,cs.data)

            #Normalization
            angle_writer.writerow([mot1,mot2,mot3,mot4,mot5,mot6,mot7])	
            mot1=normalization(float(io.data[0][0]),[-0.9,0.9],[-75,255])
            mot2=normalization(float(io.data[0][1]),[-0.9,0.9],[-120,8])
            mot3=normalization(float(io.data[0][2]),[-0.9,0.9],[-165,165])
            mot4=normalization(float(io.data[0][3]),[-0.9,0.9],[-30,130])
            mot5=normalization(float(io.data[0][4]),[-0.9,0.9],[-165,165])
            mot6=normalization(float(io.data[0][5]),[-0.9,0.9],[-80,80])
            mot7=normalization(float(io.data[0][6]),[-0.9,0.9],[-80,5])

            #if i==20:
            #    cs save->csv, npy 
	    if i == 20:
	        np.savetxt("cs20_online.csv",u_cs.data,delimiter=",")
	    if i == 100:
	        np.savetxt("cs100_online.csv",u_cs.data,delimiter=",")
            # Executes an action.
            if i>10:
                result = follow_trajectory(
                    action_service_name = ACTION_SERVICE_NAME,
                    joint_names = JOINT_NAMES,
                    positions = np.radians([mot1,mot2,mot3,mot4,mot5,mot6,mot7]),
                    time_from_start = TIME_INTERVAL
                )
            else:
                result = follow_trajectory(
                    action_service_name = ACTION_SERVICE_NAME,
                    joint_names = JOINT_NAMES,
                    positions = np.radians([6.377,-33.629,-5.559,56.225,-44.921,-16.051,-1.792]),
                    time_from_start = TIME_INTERVAL
                )
            cv2.imwrite("/home/assimilation/cameratest/test2/{:0>4d}".format(i+1) + ".jpg", cv_img)
            rospy.loginfo(result)


    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("finished.")


##
# @brief  Function for publishing action to move the arm
# @param action_service_name  ActionService' name
# @param joint_names  list of joint names
# @param positions  list of joint's goal positions(radian)
# @param time_from_start  goal time from start
# @return  ActionService's result
# @note  
def follow_trajectory(action_service_name, joint_names, positions, time_from_start):

    # Creates the SimpleActionClient.
    ac = actionlib.SimpleActionClient(action_service_name, FollowJointTrajectoryAction)

    # Waits until the action server has started up.
    ac.wait_for_server()

    # Creates a goal.
    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = rospy.Time(1.0)
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0 for i in range(len(joint_names))]
    point.accelerations = [0.0 for i in range(len(joint_names))]
    point.effort = [0.0 for i in range(len(joint_names))]
    point.time_from_start = rospy.Duration(time_from_start)
    goal.trajectory.points.append(point)

    # Sends the goal.
    ac.send_goal(goal)

    # Waits for the server.
    finished_before_timeout = ac.wait_for_result(timeout=rospy.Duration(time_from_start + 2.0))

    # Returns result.
    if finished_before_timeout:
        return ac.get_result()
    else:
        return None

if __name__ == '__main__':
    main()

