import logging
import rospy

class RospyLogger(logging.Logger):
    def __init__(self):
        #super('tello-pilot')
        pass

    def error(self, msg, *args, **kwargs):
        assert len(args) == 0 and len(kwargs) == 0
        rospy.logerr(msg)

    def warn(self, msg, *args, **kwargs):
        assert len(args) == 0 and len(kwargs) == 0
        rospy.logwarn(msg)

    def info(self, msg, *args, **kwargs):
        print("INFO!!!")
        assert len(args) == 0 and len(kwargs) == 0
        rospy.loginfo(msg)

    def debug(self, msg, *args, **kwargs):
        assert len(args) == 0 and len(kwargs) == 0
        rospy.logdebug(msg)