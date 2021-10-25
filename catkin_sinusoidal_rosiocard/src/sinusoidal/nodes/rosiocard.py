#!/usr/bin/env python
import time
import array
from math import sin, cos, atan2, pi, sqrt
from hiddriver import hiddriver
import numpy as np

import tf2_ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

ERROR_CODES = ["","Ultrasonic Module 1 Not Connected!","Ultrasonic Module 2 Not Connected!","Ultrasonic Module 1 and 2 Not Connected!","Ultrasonic Module 3 Not Connected!","Ultrasonic Module 1 and 3 Not Connected!","Ultrasonic Module 2 and 3 Not Connected!","No Ultrasonic Module Connected!!!"]

class rosiocard():
    def __init__(self,modulename="Sinusoidal rosIOcard"):
        self.riohid = hiddriver(modulename)
        rospy.init_node(modulename)
        self.modulename = modulename
        self.error_pub = rospy.Publisher('rosiocard/error', String, queue_size=10)
        if not (self.riohid.productFound):
            rospy.logerr("%s module not found"%(modulename))
            self.module_ready = False
            data = String()
            data.data = "%s module not found"%(modulename)
            self.error_pub.publish(data)
        else:
            rospy.loginfo("%s driver started"%(modulename))
            self.module_ready = True 
            self.ultrasonicPublishers = []
            for i in range(1,13):
                publisherName = "rosiocard/ultrasonic" + str(i)
                self.ultrasonicPublishers.append(rospy.Publisher(publisherName, Range, queue_size=10))
            self.infraredPublishers = []
            for i in range(1,7):
                publisherName = "rosiocard/infrared" + str(i)
                self.infraredPublishers.append(rospy.Publisher(publisherName, Range, queue_size=10))

            self.joystickPublisher = rospy.Publisher('rosiocard/joystick', Joy, queue_size=10)

            rospy.Subscriber('/rosiocard/useroutput1', Bool, self.on_user_output1)
            rospy.Subscriber('/rosiocard/useroutput2', Bool, self.on_user_output2)
            rospy.Subscriber('/rosiocard/useroutput3', Bool, self.on_user_output3)
            rospy.Subscriber('/rosiocard/openultrasonicmodule1', Bool, self.on_open_ultrasonic1)
            rospy.Subscriber('/rosiocard/openultrasonicmodule2', Bool, self.on_open_ultrasonic2)
            rospy.Subscriber('/rosiocard/openultrasonicmodule3', Bool, self.on_open_ultrasonic3)
            rospy.Subscriber('/rosiocard/useroutputPWM1', UInt8, self.on_user_output_pwm1)
            rospy.Subscriber('/rosiocard/useroutputPWM2', UInt8, self.on_user_output_pwm2)

            self.command = array.array('B',[62,0,0,0,0,0,0,0,0,60])
            self.readedData = ""

            self.globalError = 0
            self.old_globalError = 0

            self.outputPos1 = False
            self.outputPos2 = False
            self.outputPos3 = False
            self.ultrasonicmodule1 = True
            self.ultrasonicmodule2 = True
            self.ultrasonicmodule3 = True
            self.outputPwm1 = 0
            self.outputPwm2 = 0


            self.ultrasonicRanges = []
            for i in range(1,13):
                ultrasonicData = Range()
                ultrasonicData.radiation_type = 0
                ultrasonicData.field_of_view = 0.38 # 22 Degree
                ultrasonicData.min_range = 0.30 # 22 Degree
                ultrasonicData.max_range = 2.55 # 22 Degree
                ultrasonicData.header.stamp = rospy.Time.now()
                ultrasonicData.header.frame_id = "ultrasonic" + str(i)
                ultrasonicData.range = 0 
                self.ultrasonicRanges.append(ultrasonicData)


            self.infraredRanges = []
            for i in range(1,7):
                infraredData = Range()
                infraredData.radiation_type = 1
                infraredData.field_of_view = 0.1 # 5 Degree
                infraredData.min_range = 0.20 # 22 Degree
                infraredData.max_range = 1.50 # 22 Degree
                infraredData.header.stamp = rospy.Time.now()
                infraredData.header.frame_id = "infrared" + str(i)
                infraredData.range = 0 
                self.infraredRanges.append(infraredData)

            self.joystickData = Joy()
            self.joystickData.header.stamp = rospy.Time.now()
            self.joystickData.header.frame_id = "joystick"

    def on_open_ultrasonic1(self,data):
        self.ultrasonicmodule1 = data.data

    def on_open_ultrasonic2(self,data):
        self.ultrasonicmodule2 = data.data

    def on_open_ultrasonic3(self,data):
        self.ultrasonicmodule3 = data.data

    def on_user_output1(self,data):
        self.outputPos1 = data.data

    def on_user_output2(self,data):
        self.outputPos2 = data.data

    def on_user_output3(self,data):
        self.outputPos3 = data.data

    def on_user_output_pwm1(self,data):
        self.outputPwm1 = data.data

    def on_user_output_pwm2(self,data):
        self.outputPwm2 = data.data

    def send_command(self):
        self.command[0] = 62
        if(self.outputPos1):
            self.command[1]=1
        else:
            self.command[1]=0
        if(self.outputPos2):
            self.command[2]=1
        else:
            self.command[2]=0
        if(self.outputPos3):
            self.command[3]=1
        else:
            self.command[3]=0
        if(self.ultrasonicmodule1):
            self.command[4]=1
        else:
            self.command[4]=0
        if(self.ultrasonicmodule2):
            self.command[5]=1
        else:
            self.command[5]=0
        if(self.ultrasonicmodule3):
            self.command[6]=1
        else:
            self.command[6]=0
        self.command[7]=self.outputPwm1
        self.command[8]=self.outputPwm2
        self.command[9]=60
        self.riohid.write_device(self.command) 

    def read_data(self):
        databytes = self.riohid.read_device(64)            
        if(databytes == None):
            self.module_ready = False 
        else:
            if((databytes[0]==62)and(databytes[32]==60)):
                for i in range(0,12):
                    self.ultrasonicRanges[i].header.stamp = rospy.Time.now()
                    self.ultrasonicRanges[i].range = float(databytes[i+1])/100.0
                self.userButton1 = int(databytes[13])
                self.userButton2 = int(databytes[14])
                self.joystickX = int(databytes[15])+int(databytes[16])*256
                self.joystickY = int(databytes[17])+int(databytes[18])*256
                for i in range(0,6):
                    self.infraredRanges[i].header.stamp = rospy.Time.now()
                    self.infraredRanges[i].range = self.infraredRanges[i].min_range+float(databytes[i+19])*(self.infraredRanges[i].max_range)/3723.0
                self.globalError = int(databytes[31])
                if(self.globalError != self.old_globalError):
                    self.old_globalError = self.globalError
                    rospy.logwarn("%s"%(str(ERROR_CODES[self.globalError])))
                    
                for i in range(0,12):
                    self.ultrasonicPublishers[i].publish(self.ultrasonicRanges[i])
                    
                for i in range(0,6):
                    self.infraredPublishers[i].publish(self.infraredRanges[i])
                self.joystickData.axes = [float(self.joystickX), float(self.joystickY)]
                self.joystickData.buttons = [self.userButton1, self.userButton2]
                self.joystickPublisher.publish(self.joystickData)
            self.send_command()
            

if __name__ == '__main__':
    rio = rosiocard("Sinusoidal rosIOcard")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if(rio.module_ready):
            rio.read_data()
        else:
            time.sleep(5)
            del rio
            rio = rosiocard("Sinusoidal rosIOcard")
        rate.sleep()
