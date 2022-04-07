from getmac import get_mac_address
import socket
import time
import rospy


class MACManager:
    """ Based on Daniel Röder's code
    """
    def __init__(self) -> None:
        self.local_ip = socket.gethostbyname(socket.gethostname())
        self.cache = {}

    def get(self, tello_mac:str):

        tello_mac = tello_mac.lower()

        if tello_mac in self.cache:
            return self.cache[tello_mac]

        search_ip_retries = 3
        for i in range(search_ip_retries):
            start = 100
            for i in range(255-start):
                ip = ip=f"192.168.0.{start}"
                if ip != self.local_ip:
                    mac = get_mac_address(ip=ip)
                    if mac == None:
                        continue

                    self.cache[mac] = ip

                    if mac == tello_mac:
                        rospy.loginfo(f"Found Tello IP: 192.168.0.{start}")
                        return f"192.168.0.{start}"
                start += 1
            time.sleep(3)
            rospy.loginfo("Couldn't find Tello in local network. Trying again...")