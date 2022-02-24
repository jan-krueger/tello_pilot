#!/usr/bin/env python3

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
        self.camera_topic = rospy.get_param(rospy.get_namespace() + rospy.get_name() + '/camera')
        self.image_subscriber = rospy.Subscriber(self.camera_topic, Image, self.process_image)

        self.processing_lock = Lock()

        self.processing_timer = rospy.Timer(rospy.Duration(1), self.process_images_callback)

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
            cv.imshow("[CameraDelay] Marker | %s" % self.camera_topic, tag)
            key = cv.waitKey(500)

            if key == 27: # 27 -> ESC
                cv.destroyAllWindows()
                break

            rospy.Duration(5)
        print("")

    def process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)

        with self.processing_lock:
            self.image_queue.append({'frame_received': msg.header.stamp,
                'frame': frame,
                'marker_display_time': self.current_marker_time, 
                'marker_id': self.aruco_id, 'processed': False})

    def process_images_callback(self, _):
        with self.processing_lock:
            processed_ids = set()
            for event in self.image_queue:
                
                if event['processed'] == True:
                    continue
                event['processed'] = True

                _, ids, _ = cv.aruco.detectMarkers(event['frame'], self.aruco_dictionary)

                if event['marker_id'] in processed_ids:
                    continue
                
                if ids is not None:
                    detected_id = ids[0][0]
                    if detected_id == event['marker_id']:
                        processed_ids.add(event['marker_id'])
                        self.time_deltas.append((event['frame_received'] - event['marker_display_time']).to_sec())
            if len(self.time_deltas) > 0:
                print("detected markers %d | avg. dt: %.4f [s] | std. dev.: %.4f" %
                        (len(self.time_deltas), np.mean(self.time_deltas), np.std(self.time_deltas)),
                        end='\r')
            else:
                print("buffer is still empty...", end='\r')


def main():
    rospy.init_node('camera_delay_measurement_node', anonymous=True)
    CameraDelayMeasurementNode()
    rospy.spin()


if __name__ == '__main__':
    main()  