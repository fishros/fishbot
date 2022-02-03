# -*- coding: utf-8 -*-
from queue import Queue
import threading
import time
import struct

from fishbot_controller.tool import check_sum

MIN_FRAME_LEN = 6 #最小的帧长度

class BaseFrame():
    def __init__(self,target= 0,command= 0,len= 0, data=b'') -> None:
        self.target = target
        self.command = command
        self.data_len = len
        self.data = data

    def getFrame(self):
        check_result = check_sum.uchar_checksum(self.data)
        # print("check result:",check_result)
        frame = b'\x7D' \
                        +struct.pack("bbb",self.target,self.command,self.data_len) \
                        +self.data \
                        +struct.pack('B',check_result) \
                +b'\x7E'
        return frame

    @staticmethod
    def GenFrame(frame):
        """
        通过bytes产生一个BaseFrame对象
        """
        if len(frame) < MIN_FRAME_LEN:
            return None
        # parse data
        base_frame = BaseFrame()
        base_frame.target = struct.unpack('B',frame[1:2])[0]
        base_frame.command = struct.unpack('B',frame[2:3])[0]
        base_frame.data_len = struct.unpack('B',frame[3:4])[0]
        base_frame.data = frame[4:4+base_frame.data_len]
        # calculate sum 
        check_result = check_sum.uchar_checksum(base_frame.data)
        if check_result == frame[-2]:
            return base_frame   
        print("check sum fail:frame",frame)
        print("check result:",check_result)

    @staticmethod
    def GenBaseFrame(target,command,data_len,data):
        """
        通过bytes产生一个BaseFrame对象
        """
        base_frame = BaseFrame()
        base_frame.target = target
        base_frame.command = command
        base_frame.data_len = data_len
        base_frame.data = data

        return base_frame
    