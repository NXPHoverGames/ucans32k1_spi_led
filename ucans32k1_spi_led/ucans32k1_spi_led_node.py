#!/usr/bin/env python3
import os
import sys
import copy
import re
import importlib
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from  sensor_msgs.msg import Image
from std_msgs.msg import String
from canfd_msgs.msg import OpenCyphalMessage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile

if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))

class UCANS32K1SPILEDNode(Node):

    def __init__(self):

        super().__init__("ucans32k1_spi_led_node")

        image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='SPI LED image topic.')

        led_pattern_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='SPI LED pattern topic string.')

        max_leds_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Max number of LEDs to drive.')

        led_brightness_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='LED Brightness [0,31].')
        

        self.declare_parameter("led_image_topic", "led_image", 
            image_topic_descriptor)
        
        self.declare_parameter("led_pattern_topic", "led_pattern", 
            led_pattern_topic_descriptor)

        self.declare_parameter("max_leds", 64, 
            max_leds_descriptor)

        self.declare_parameter("led_brightness", 10, 
            led_brightness_descriptor)


        self.patternTopic = self.get_parameter("led_pattern_topic").value

        self.imageTopic = self.get_parameter("led_image_topic").value

        self.maxLEDs = int(self.get_parameter("max_leds").value)

        self.Brightness = int(self.get_parameter("led_brightness").value)

        if self.Brightness > 31:
            self.Brightness = 31
        elif self.Brightness < 0:
            self.Brightness = 0

        if self.maxLEDs > 1000:
            self.maxLEDs = 1000
        elif self.maxLEDs < 1:
            self.maxLEDs = 1

        self.previousRGBHex=np.array([], dtype=np.uint32)

        self.previousPattern="OFF"

        self.pubMaxHz=500

        self.AllowRepeats = True

        self.extraPath = os.path.realpath(os.path.relpath(os.path.join(os.path.realpath(__file__).replace("ucans32k1_spi_led_node.py",""),"../extras")))


        self.BackwardYellowImg=cv2.imread(os.path.join(self.extraPath,'YellowDirection.png'), cv2.IMREAD_COLOR)
        self.LeftYellowImg=cv2.rotate(self.BackwardYellowImg, cv2.ROTATE_90_CLOCKWISE)
        self.RightYellowImg=cv2.rotate(self.BackwardYellowImg, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.ForwardYellowImg=cv2.rotate(self.BackwardYellowImg, cv2.ROTATE_180)

        self.BackwardGreenImg=cv2.imread(os.path.join(self.extraPath,'GreenDirection.png'), cv2.IMREAD_COLOR)
        self.LeftGreenImg=cv2.rotate(self.BackwardGreenImg, cv2.ROTATE_90_CLOCKWISE)
        self.RightGreenImg=cv2.rotate(self.BackwardGreenImg, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.ForwardGreenImg=cv2.rotate(self.BackwardGreenImg, cv2.ROTATE_180)


        self.ReverseImg=cv2.imread(os.path.join(self.extraPath,'Reverse.png'), cv2.IMREAD_COLOR)

        self.HazardImg=cv2.imread(os.path.join(self.extraPath,'Hazard.png'), cv2.IMREAD_COLOR)

        self.StopImg=cv2.imread(os.path.join(self.extraPath,'Stop.png'), cv2.IMREAD_COLOR)

        self.SadfaceImg=cv2.imread(os.path.join(self.extraPath,'Sadface.png'), cv2.IMREAD_COLOR)

        self.NImg=cv2.imread(os.path.join(self.extraPath,'N.png'), cv2.IMREAD_COLOR)

        self.XImg=cv2.imread(os.path.join(self.extraPath,'X.png'), cv2.IMREAD_COLOR)

        self.PImg=cv2.imread(os.path.join(self.extraPath,'P.png'), cv2.IMREAD_COLOR)

        self.LowPowerImg=cv2.imread(os.path.join(self.extraPath,'LowPower.png'), cv2.IMREAD_COLOR)

        self.UltraLowPowerImg=cv2.imread(os.path.join(self.extraPath,'UltraLowPower.png'), cv2.IMREAD_COLOR)

        self.PersonImg=cv2.imread(os.path.join(self.extraPath,'Person.png'), cv2.IMREAD_COLOR)

        self.WhiteImg=cv2.imread(os.path.join(self.extraPath,'White.png'), cv2.IMREAD_COLOR)



        #setup CvBridge
        self.bridge = CvBridge()
        
        self.InitTime = int(round(self.get_clock().now().nanoseconds/1000.0))
        
        self.CounterCyphalMsg = 0

        self.PubCyphal = self.create_publisher(OpenCyphalMessage, 'CyphalTransmitFrame', 0)

        self.patternSub = self.create_subscription(String, 
            '{:s}'.format(self.patternTopic), 
            self.patternMatch, 10)

        self.imageSub = self.create_subscription(Image, 
            '{:s}'.format(self.imageTopic), 
            self.imageCallback, 
            qos_profile_sensor_data)

    
    def imageCallback(self, data):
        passedImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if passedImage.shape[0]*passedImage.shape[1] <= self.maxLEDs:
            self.BGR2RGBHex(passedImage)
        else:
            print('Image pixel size passed: {:d}, Max allowed: {:d}'.format(passedImage.shape[0]*passedImage.shape[1], self.maxLEDs))
        return


    def patternMatch(self, patternString):
        if patternString.data.upper() !=  self.previousPattern.upper() or self.AllowRepeats:
            if self.maxLEDs >= 64:
                if patternString.data.upper() == "WHITE":
                    self.BGR2RGBHex(self.WhiteImg)
                elif patternString.data.upper() == "LOWPOWER":
                    self.BGR2RGBHex(self.LowPowerImg)
                elif patternString.data.upper() == "ULTRALOWPOWER":
                    self.BGR2RGBHex(self.UltraLowPowerImg)
                elif patternString.data.upper() == "BACKWARDYELLOW":
                    self.BGR2RGBHex(self.BackwardYellowImg)
                elif patternString.data.upper() == "BACKWARDGREEN":
                    self.BGR2RGBHex(self.BackwardGreenImg)
                elif patternString.data.upper() == "FORWARDYELLOW":
                    self.BGR2RGBHex(self.ForwardYellowImg)
                elif patternString.data.upper() == "FORWARDGREEN":
                    self.BGR2RGBHex(self.ForwardGreenImg)
                elif patternString.data.upper() == "LEFTYELLOW":
                    self.BGR2RGBHex(self.LeftYellowImg)
                elif patternString.data.upper() == "LEFTGREEN":
                    self.BGR2RGBHex(self.LeftGreenImg)
                elif patternString.data.upper() == "RIGHTYELLOW":
                    self.BGR2RGBHex(self.RightYellowImg)
                elif patternString.data.upper() == "RIGHTGREEN":
                    self.BGR2RGBHex(self.RightGreenImg)
                elif patternString.data.upper() == "REVERSE":
                    self.BGR2RGBHex(self.ReverseImg)
                elif patternString.data.upper() == "HAZARD":
                    self.BGR2RGBHex(self.HazardImg)
                elif patternString.data.upper() == "STOP":
                    self.BGR2RGBHex(self.StopImg)
                elif patternString.data.upper() == "SADFACE":
                    self.BGR2RGBHex(self.SadfaceImg)
                elif patternString.data.upper() == "N":
                    self.BGR2RGBHex(self.NImg)
                elif patternString.data.upper() == "X":
                    self.BGR2RGBHex(self.XImg)
                elif patternString.data.upper() == "P":
                    self.BGR2RGBHex(self.PImg)
                elif patternString.data.upper() == "PERSON":
                    self.BGR2RGBHex(self.PersonImg)
        return



    def BGR2RGBHex(self, image):
        RGBHex=np.array([0], dtype=np.uint32)
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                RGBHex=np.append(RGBHex, np.uint32((image[y][image.shape[1]-x-1][0] << 16) + (image[y][image.shape[1]-x-1][1] << 8) + image[y][image.shape[1]-x-1][2]))
        
        if not np.array_equal(self.previousRGBHex,RGBHex) or self.AllowRepeats:
            self.previousRGBHex=RGBHex
            NumberLeds = len(RGBHex)
            NumberGroups = int(np.ceil(NumberLeds/10.0))
            for OffsetGroup in range(NumberGroups):
                LedValArray=RGBHex[OffsetGroup*10:np.min([(OffsetGroup+1)*10,NumberLeds])]
                msg = OpenCyphalMessage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.priority = int(4)
                msg.is_annonymous = False
                msg.subject_id = int(501)
                msg.source_node_id = int(96)
                msg.data = self.ConvertDataSPILED(OffsetGroup, NumberLeds, self.Brightness, LedValArray)
                msg.crc= int(224+(self.CounterCyphalMsg%32))
                time.sleep(1.0/self.pubMaxHz)
                self.PubCyphal.publish(msg)
                self.CounterCyphalMsg += 1
        return
            


    def ConvertDataSPILED(self, OffsetGroup, NumberLeds, Brightness, LedValArray):
        DataArray=np.array([], dtype=np.uint8)

        TimeSinceInit = int(round(self.get_clock().now().nanoseconds/1000.0))-self.InitTime
        for i in range(8):
            DataArray = np.append(DataArray,
                    [np.uint8((TimeSinceInit >> i*8) & 255)], 
                    axis=0)
        if len(LedValArray) <= 10:
            for Led in range(len(LedValArray)):
                useBrightness = (Brightness & 0x1F) + 0xE0
                DataArray = np.append(DataArray, [
                            np.uint8((LedValArray[Led] >> 16) & 0xFF), #RED
                            np.uint8((LedValArray[Led] >> 8) & 0xFF), #GREEN
                            np.uint8(LedValArray[Led] & 0xFF), #BLUE
                            np.uint8(useBrightness)
                            ], axis=0)
            if len(LedValArray) < 10: 
                for NoVal in range(10-len(LedValArray)):
                    DataArray = np.append(DataArray, [
                            np.uint8(0),np.uint8(0),np.uint8(0),np.uint8(0)], axis=0)

        else:
            print("LedValArray too large, max is 10")
        
        DataArray = np.append(DataArray,
                            [np.uint8(NumberLeds & 255),
                             np.uint8(NumberLeds >> 8)], axis=0)
        DataArray = np.append(DataArray,[np.uint8(OffsetGroup & 255)], axis=0)
        
        while len(DataArray) < 63:
            DataArray = np.append(DataArray, 
                               [np.uint8(0)], axis=0)
        return DataArray

def main(args=None):
    rclpy.init(args=args)
    node = UCANS32K1SPILEDNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

