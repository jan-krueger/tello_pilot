#!/usr/bin/env python3

from imp import lock_held
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np

import collections
from threading import Lock


class CameraDelayMeasurementNode:

    def __init__(self) -> None:

        self.image_queue = collections.deque(maxlen=120)

        self.bridge = CvBridge()
        self.camera_topic = rospy.get_param('/camera_delay_measurement_node/camera')
        self.image_subscriber = rospy.Subscriber(self.camera_topic, Image, self.process_image)

        self.processing_lock = Lock()

        self.processing_timer = rospy.Timer(rospy.Duration(1), self.process_images_callback)
        self.arcuo_display = rospy.Timer(rospy.Duration(1), self.arcuo_display_callback)

        self.aruco_dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.aruco_id = 0
        self.aruco_dictionary_size = 50 # NOTE: HAS TO CHANGE IF YOU CHANGE self.aruco_dictionary
        self.current_marker_time = rospy.Time.now()

        self.time_deltas = []

        while rospy.is_shutdown() is False:
            tag = np.zeros((300, 300, 1), dtype="uint8")
            self.aruco_id = (self.aruco_id + 1) % self.aruco_dictionary_size
            cv.aruco.drawMarker(self.aruco_dictionary, self.aruco_id, 300, tag, 1)
            self.current_marker_time = rospy.Time.now()
            cv.imshow("[CameraDelay] Marker", tag)
            key = cv.waitKey(1000)

            if key == 27:
                cv.destroyAllWindows()
                break
            rospy.Duration(5)
            cv.destroyAllWindows()

    def process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)

        self.processing_lock.acquire()
        self.image_queue.append({'frame_received': msg.header.stamp,
            'frame': frame,
            'marker_display_time': self.current_marker_time, 
            'marker_id': self.aruco_id})
        self.processing_lock.release()

    def process_images_callback(self, _):
        self.processing_lock.acquire()
        processed_ids = set()
        for event in self.image_queue:
            _, ids, _ = cv.aruco.detectMarkers(event['frame'], self.aruco_dictionary)

            if event['marker_id'] in processed_ids:
                continue
            
            if ids is not None:
                detected_id = ids[0][0]
                if detected_id == event['marker_id']:
                    processed_ids.add(event['marker_id'])
                    self.time_deltas.append((event['frame_received'] - event['marker_display_time']).to_sec())
        self.processing_lock.release()
        print("detected markers %d | avg. dt: %.4f [s] | std. dev.: %.4f" %
                    (len(self.time_deltas), np.mean(self.time_deltas), np.std(self.time_deltas)))

    def arcuo_display_callback(self, event):
        pass
        

def main():
    rospy.init_node('camera_delay_measurement_node')
    CameraDelayMeasurementNode()
    rospy.spin()


if __name__ == '__main__':
    main()  