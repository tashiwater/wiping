#!/usr/bin/env python
import sys
import rospy
import serial
from std_msgs.msg import Float32MultiArray
import time
import numpy as np


class TouchenceSensor(object):
    def __init__(self, args):
        print 'Open touchence sensor'
        self.ser = serial.Serial(args[0], 115200, timeout=0.1)
        self.sensor_initialize()

        self.sensor_data = Float32MultiArray()
        self.sensor_data.data = np.zeros(12, dtype=np.float32)
        self.sensor_pub = rospy.Publisher('/touchence/sensor_data', Float32MultiArray, queue_size=1)


        
    def sensor_initialize(self):

        self.ser.write('r')
        print "reset the sensor"
        print "return fron sensor"
        print self.ser.readline()
        
        print "send number of sensor"
        self.ser.write('03')
        print self.ser.readline()
        print self.ser.readline()
        
        print "send sensor ID"
        self.ser.write('01')
        print self.ser.readline()
        print self.ser.readline()
        
        print "send sensor ID"
        self.ser.write('06')
        print self.ser.readline()
        print self.ser.readline()    

        print "send sensor ID"
        self.ser.write('03')
        print self.ser.readline()
        print self.ser.readline()    
    
        print "change connection time"
        self.ser.write("@020101\r\n")
        print self.ser.readline()

        self.ser.flushInput()
        self.ser.flushInput()


    def bit_to_voltage(self, data):
        voltage = np.array([ int(data[4:8], 16), int(data[8:12], 16),\
                             int(data[12:16], 16), int(data[16:20], 16) ])
        
        return voltage.astype(np.float32)/1023 * 3.3


    def get_value(self):
        self.ser.write('o')        
        self.sensor_data.data[0:4] = self.bit_to_voltage(self.ser.readline())
        self.sensor_data.data[4:8] = self.bit_to_voltage(self.ser.readline())
        self.sensor_data.data[8:12] = self.bit_to_voltage(self.ser.readline())

        return self.sensor_data


    def shutdown(self):
        print "close touchence sensor and serial port"
        self.ser.write('l')
        self.ser.close()
    

if __name__ == "__main__":
    
    rospy.init_node('touchence', anonymous=True)

    args = [rospy.get_param('~device', '/dev/ttyUSB1'),\
            ]

    print 'load parameter: Device=%s' % args[0]

    sensor = TouchenceSensor(args)

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        sensor_value = sensor.get_value()
        #print sensor_value
        sensor.sensor_pub.publish(sensor_value)

        r.sleep()


    sensor.shutdown()
