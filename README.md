## Tello Pilot

### Connect Drone to Access Point

    1. Turn on your Tello, and connect to its WiFi.
    2. Run the following ROS node: `rosrun tello_pilot ap_setup_node.py _ssid:="TP-Link_9975" _password:="62922623"`   
    3. The Tello should restart, and now successfully be connected to the AP.
    4. If you want to hard-reset the drone turn it on until it flashes regulary then hold the power button ~5s until it turns of. Let go of the power button. The drone should automatically boot up again after a few seconds.

### Maximum Yaw Velocity

    Based on basic experiments, the Tello seems to be able to yaw at a maximum rate of `pi/2 rad/s`. While you can provide higher values in the `TwistStamped` message to the `cmd_vel` topic all values are normalised with respect to the aformentioned maximum value.