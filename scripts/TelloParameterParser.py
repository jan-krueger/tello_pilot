
from DJITelloPy.djitellopy.tello import Tello

class TelloParameterParser:

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
