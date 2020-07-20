#!/usr/bin/env python
import rospy
import sys
from os import path 
from sensor_msgs.msg import JointState
import torobo_collision_detector.check_collision_client
from torobo_driver import servo_off_client
from torobo_driver import get_servo_state_client
from torobo_common import repeat_get_param 

class SelfCollisionChecker(object):
    def __init__(self, sim, rateHz):
        self.sim_ = sim
        self.rateHz_ = rateHz
        self.nameSpace_ = rospy.get_namespace()
        self.jointState_ = JointState()
        self.controllerListNameSpace = rospy.get_namespace() + "move_group/controller_list"
        self.controller_dict_ = self.get_controller_dict(self.controllerListNameSpace)
        self.servo_off_client_ = self.create_servo_off_service_client()
        self.get_servo_state_client_ = self.create_get_servo_state_service_client()
        self.sub_ = rospy.Subscriber(rospy.get_namespace() + 'joint_states', JointState, self.callback, queue_size=1, tcp_nodelay=True)

    def get_controller_dict(self, controllerListNameSpace):
        dic = {}
        controllerNames = self.get_controller_names(controllerListNameSpace)
        for name in controllerNames:
            jointNameIdDict = self.get_joint_name_id_dict(controllerListNameSpace, name)
            dic[name] = jointNameIdDict
        return dic

    def get_joint_name_id_dict(self, controllerListNameSpace, controllerName):
        dic = {}
        controller_list = repeat_get_param.get_param(controllerListNameSpace, retry_num=50)
        if (controller_list is None):
            return dic
        id = 0
        for l in controller_list:
            if controllerName in l["name"]:
                for j in l["joints"]:
                    dic[j] = id
                    id += 1
        return dic

    def get_controller_names(self, controllerListNameSpace):
        names = []
        rospy.sleep(0.5)
        controller_list = repeat_get_param.get_param(controllerListNameSpace, retry_num=50)
        if (controller_list is None):
            return names
        tempns = self.nameSpace_.replace("/", "")
        for l in controller_list:
            fullname = l["name"]
            name = fullname.replace(tempns, "")
            name = name.lstrip("/")
            names.append(name)
        return names
    
    def create_servo_off_service_client(self):
        client = {}
        for (name, dic) in self.controller_dict_.items():
            client[name] = servo_off_client.ServoOffClient(self.nameSpace_ + name)
        return client

    def create_get_servo_state_service_client(self):
        client = {}
        for (name, dic) in self.controller_dict_.items():
            client[name] = get_servo_state_client.GetServoStateClient(self.nameSpace_ + name)
        return client

    def callback(self, jointState):
        self.jointState_ = jointState

    def collision_check(self, jointState):
        return torobo_collision_detector.check_collision_client.call_service(self.nameSpace_, jointState)

    def call_servo_off_service_all_controllers(self):
        for (name, client) in self.servo_off_client_.items():
            client.call_service("all")

    def call_get_servo_state_service_all_controllers(self):
        is_servo_on = []
        for (name, client) in self.get_servo_state_client_.items():
            ret = client.call_service("all")
            if ret is None:
                continue
            is_servo_on.extend(ret)
        return is_servo_on
            
    def run(self):
        rospy.loginfo("self-collision checker standby!")
        r = rospy.Rate(self.rateHz_)
        while not rospy.is_shutdown():
            if(self.collision_check(self.jointState_)):
                # Simulation
                if(self.sim_):
                    rospy.loginfo("is colliding!")
                # Real hardware
                else:
                    is_servo_on = self.call_get_servo_state_service_all_controllers()
                    if(any(is_servo_on)):
                        rospy.loginfo("is colliding! -> servo off all joints")
                        self.call_servo_off_service_all_controllers()
                    rospy.sleep(0.1)
            r.sleep()

if __name__ == '__main__':
    nodeName = path.splitext(path.basename(__file__))[0]
    rospy.init_node(nodeName)

    sim = rospy.get_param("~sim")
    rateHz = rospy.get_param("~rateHz")
    try:
        scc = SelfCollisionChecker(sim, rateHz)
        scc.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass