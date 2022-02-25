#!/usr/bin/env python3

import rospy

from DJITelloPy.djitellopy.tello import Tello

class ApSetupNode:

    def __init__(self):
        self.tello = Tello()
        self.tello.connect()
        self.tello.connect_to_wifi(rospy.get_param('~ssid'), str(rospy.get_param('~password')))
        rospy.loginfo("Done. This node can be closed now!")

def main():
    rospy.init_node('ap_setup_node')
    ApSetupNode()
    rospy.spin()


if __name__ == '__main__':
    main()