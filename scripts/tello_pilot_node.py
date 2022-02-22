#!/usr/bin/env python3

import logging
from time import sleep
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from tello_pilot.msg import CameraDirection

import cv2 as cv
from RospyLogger import RospyLogger
from TelloParameterParser import TelloParameterParser

from cv_bridge import CvBridge
from threading import Thread

from DJITelloPy.djitellopy.tello import Tello, TelloException

class TelloNode:

    def __init__(self):
        print(rospy.get_param('~camera_fps'))
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()

        self.camera_direction = Tello.CAMERA_FORWARD
        self.tello.set_video_direction(self.camera_direction)
        self.tello.set_video_fps(
            TelloParameterParser.param_camera_fps(rospy.get_param('~camera_fps')))
        self.tello.set_video_bitrate(
            TelloParameterParser.param_camera_bitrate(rospy.get_param('~camera_bitrate')))
        #self.tello_ip = rospy.get_param('~tello_ip', '192.168.10.1')
        self.bridge = CvBridge()

        # Connect to drone
        Tello.LOGGER = RospyLogger()
        self.takeoff_subscriber = rospy.Subscriber('takeoff', Empty, self.cmd_takeoff)
        self.land_subscriber = rospy.Subscriber('land', Empty, self.cmd_land)
        self.camera_direction_subscriber = rospy.Subscriber('camera/direction', CameraDirection, self.cmd_camera_direction),

        self.image_raw_publisher = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        self.video_thread = Thread(target=self.pub_image_raw)
        self.frame_read = self.tello.get_frame_read()
        self.video_thread.start()

    def pub_image_raw(self):
        while True:
            img_msg = self.bridge.cv2_to_imgmsg(self.frame_read.frame, 'rgb8')
            img_msg.header.stamp = rospy.Time.now()
            self.image_raw_publisher.publish(img_msg)
            sleep(1 / 25.)

    def cmd_takeoff(self, msg):
        self.tello.takeoff()

    def cmd_land(self, msg):
        self.tello.land()

    def cmd_camera_direction(self, msg):
        if msg.forward:
            if self.camera_direction is not Tello.CAMERA_FORWARD:
                self.tello.set_video_direction(Tello.CAMERA_FORWARD)
                self.camera_direction = Tello.CAMERA_FORWARD
        elif self.camera_direction is not Tello.CAMERA_DOWNWARD: 
            self.tello.set_video_direction(Tello.CAMERA_DOWNWARD)
            self.camera_direction = Tello.CAMERA_DOWNWARD

    def __del__(self):
        self.video_thread.join()


def main():
    rospy.init_node('tello_pilot_node')
    TelloNode()
    rospy.spin()


if __name__ == '__main__':
    main()