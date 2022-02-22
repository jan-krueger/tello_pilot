#!/usr/bin/env python3
import enum
from threading import Thread
import rospy
from DJITelloPy.djitellopy.swarm import TelloSwarm
import cv2
rospy.init_node('test_node')

swarm = TelloSwarm.fromIps([
    "192.168.0.100",
    "192.168.0.102",
])

swarm.connect()
swarm.streamon()

for (i, tello) in enumerate(swarm):
    frame_read = tello.get_frame_read()
    def videoRecorder():
        # create a VideoWrite object, recoring to ./video.avi
        # 创建一个VideoWrite对象，存储画面至./video.avi

        while True:
            cv2.imshow("%d test" % i, frame_read.frame)
            cv2.waitKey(-1)


    # we need to run the recorder in a seperate thread, otherwise blocking options
    #  would prevent frames from getting added to the video
    # 我们需要在另一个线程中记录画面视频文件，否则其他的阻塞操作会阻止画面记录
    recorder = Thread   (target=videoRecorder)
    recorder.start()

rospy.spin()