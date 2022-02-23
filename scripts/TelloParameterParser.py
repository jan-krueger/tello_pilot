
from numpy import empty
from DJITelloPy.djitellopy.tello import Tello

class TelloParameterParser:

    @staticmethod
    def param_tello_list(input: str):
        input = input.strip()

        if len(input) == 0:
            return (True, {'default': None})

        prefix_list = input.split(',')
        tello_list = {}
        
        for prefix in prefix_list:
            if prefix not in tello_list:
                tello_list[prefix] = None
            else:
                raise Exception("Duplicate prefix in tello_list. %s" % prefix)

        return (len(tello_list) == 0, tello_list)

    @staticmethod
    def param_camera_bitrate(input: str):
        input = input.lower()

        if input == '1':
            return Tello.BITRATE_1MBPS
        elif input == '2':
            return Tello.BITRATE_2MBPS  
        elif input == '3':
            return Tello.BITRATE_3MBPS  
        elif input == '4':
            return Tello.BITRATE_4MBPS  
        elif input == '5':
            return Tello.BITRATE_5MBPS  
        else:
            return Tello.BITRATE_AUTO

    @staticmethod
    def param_camera_fps(input: int):
        if input == 5:
            return Tello.FPS_5
        elif input == 15:
            return Tello.FPS_15
        else:
            return Tello.FPS_30

    @staticmethod
    def param_camera_resolution(input: str):
        if input == '480P':
            return Tello.RESOLUTION_480P
        else:
            return Tello.RESOLUTION_720P
