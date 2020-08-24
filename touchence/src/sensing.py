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
    
    def init(self):
        self._sensor_num = 4 #int(input("num sensor: "))
        self.sensor_initialize()

        self.sensor_data = Float32MultiArray()
        self.sensor_data.data = np.zeros(self._sensor_num * 4, dtype=np.float32)
        self.sensor_pub = rospy.Publisher('/touchence/sensor_data', Float32MultiArray, queue_size=1)

    def setID(self):
        strs = "@0205{:02d}\r\n".format(input("set ID: "))
        self.ser.write(strs)
        print self.ser.readline()
        
    def sensor_initialize(self):

        self.ser.write('r')
        print "reset the sensor"
        print "return from sensor"
        print self.ser.readline()
        
        print "send number of sensor"
        self.ser.write('0{}'.format(self._sensor_num))
        print self.ser.readline()
        print self.ser.readline()
        # a = int(input())
        # num_list = [3,4,6]
        num_list = [i+1 for i in range(self._sensor_num)]
        # num_list = [int(input("ID: ")) for i in range(self._sensor_num)]
        for i in num_list:
            print "send sensor ID"
            num = i
            self.send_read("{:02d}".format(num))
    
        print "change connection time"
        self.ser.write("@020101\r\n")
        print self.ser.readline()

        self.ser.flushInput()
        self.ser.flushInput()

    def send_read(self, str):
        self.ser.write(str)
        print("ret" + self.ser.readline())
        print("ret" + self.ser.readline() )   

    def bit_to_voltage(self, data):
        voltage = np.array([ int(data[4:8], 16), int(data[8:12], 16),\
                             int(data[12:16], 16), int(data[16:20], 16) ])
        data = voltage.astype(np.float32)/1023 * 3.3
        # print(data)
        return data 


    def get_value(self):
        self.ser.write('o')    
        for i in range(self._sensor_num):    
            posi = i * 4
            # data = ""
            # while data == "":
            data = self.ser.readline()
            if data == "":
                continue
            # print(i)
            self.sensor_data.data[posi:posi+4] = self.bit_to_voltage(data)
        # self.sensor_data.data[4:8] = self.bit_to_voltage(self.ser.readline())
        # self.sensor_data.data[8:12] = self.bit_to_voltage(self.ser.readline())

        return self.sensor_data


    def shutdown(self):
        print "close touchence sensor and serial port"
        self.ser.write('l')
        self.ser.close()
    
    def __del__(self):
        self.shutdown()
        

if __name__ == "__main__":
    rospy.init_node('touchence', anonymous=True)

    args = [rospy.get_param('~device', '/dev/ttyUSB1'),\
            ]

    print 'load parameter: Device=%s' % args[0]

    sensor = TouchenceSensor(args)
    sensor.init()
    # sensor.setID() #if you want to set new ID

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        sensor_value = sensor.get_value()
        #print sensor_value
        sensor.sensor_pub.publish(sensor_value)

        r.sleep()


    sensor.shutdown()
