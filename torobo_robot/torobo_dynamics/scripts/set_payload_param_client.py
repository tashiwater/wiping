#!/usr/bin/env python
## -*- coding: utf-8 -*-
import rospy
from torobo_msgs.srv import SetPayloadParam
import sys
from os import path 

def call_service(ns, prefix, mass, com,inertiaElem):
    try:
        if (ns[-1] != "/"):
            ns += "/"
        rospy.loginfo('set_payload_param service client is called')
        service_name = ns + '/' + prefix + '_controller/set_payload_param'
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, SetPayloadParam)
        response = service(prefix, mass, com,inertiaElem)
        ret = "failed"
        if response.success:
            ret = "succeeded"
        rospy.loginfo('set_payload_param [%s, %f, %s, %s] is %s' % (prefix, mass, map(str, com), map(str, inertiaElem), ret))
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s" % e)
        return
    return None

def test():
    nodeName = path.splitext(path.basename(__file__))[0]
    rospy.init_node(nodeName)

    com = [0.0,0.0, 0.046]
    inertiaElem = [1.104105e-3 ,2.054322e-5 ,-5.411983e-6 ,7.765804e-4 ,3.247528e-5 ,6.346240e-4]

    call_service('torobo', 'right_arm', 0.76 ,com ,inertiaElem) 
    call_service('torobo', 'left_arm', 0.76 ,com ,inertiaElem) 

if __name__ == '__main__':
    if (len(sys.argv) > 3):
        ns = sys.argv[1]
        prefix = sys.argv[2]
        mass = float(sys.argv[3])
        com = []
        inertiaElem = []
        if (len(sys.argv) > 6):
            com = [float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])]
        if (len(sys.argv) > 12):
            inertiaElem = [float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]), float(sys.argv[12])]
        nodeName = path.splitext(path.basename(__file__))[0]
        rospy.init_node(nodeName)
        call_service(ns, prefix, mass, com, inertiaElem)
    else:
        print "invalid args"
