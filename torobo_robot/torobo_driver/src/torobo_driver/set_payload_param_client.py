#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import SetPayloadParam
import sys
from os import path 

class SetPayloadParamClient(object):
    def __init__(self, ns):
        if (ns[-1] != "/"):
            ns += "/"
        self._ns = ns
        self.service = rospy.ServiceProxy(self._ns + 'set_payload_param', SetPayloadParam)

    def call_service(self, mass, massCenter=[0.0, 0.0, 0.0]):
        rospy.loginfo('set_payload_param service client is called')
        try:
            name = ""
            inertiaElem = []
            response = self.service(name, mass, massCenter, inertiaElem)
            ret = "failed"
            if response.success:
                ret = "succeeded"
            rospy.loginfo('set_payload_param [%f, %s] is %s' % (mass, map(str,massCenter), ret))
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    if (len(sys.argv) > 3):
        ns = sys.argv[1]
        mass = float(sys.argv[2])
        com = []
        if (len(sys.argv) > 5):
            com = [float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])]

        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        client = SetPayloadParamClient(ns)
        client.call_service(mass, com)
    else:
        print "invalid args"