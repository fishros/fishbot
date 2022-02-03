'''
作者: 小鱼
公众号: 鱼香ROS
QQ交流群: 2642868461
描述: file content
'''
# -*- coding: utf-8 -*-
import socket
from queue import Queue
import threading
import time

server = None

class UdpServer:
    def __init__(self,queue=Queue(),port=3334):
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.event = threading.Event()
        
        self.frame_queue = queue
        self.addr = None

        self.__enter__()

    def __enter__(self):
        try:
            self.socket.bind(('', self.port))
        except socket.error as e:
            print('Bind failed:{}'.format(e))
            raise
        # self.socket.listen()
        print('Starting server on port={} ip={}'.format(self.port,"0.0.0.0"))
        # 启动线程接受连接
        self.server_thread = threading.Thread(target=self.run_server,name='run_server')
        self.server_thread.start()
        return self

    def send_frame(self,frame):
        # print("Send to %s, frame: %s",self.addr,frame)
        self.socket.sendto(frame,self.addr)

    def run_server(self):
        print("server start....")
        frames = b''
        count = 0
        last_time = time.time()
        start =  time.time()
        while not self.event.is_set():
            data, self.addr = self.socket.recvfrom(1024)
            if not data: continue
            frames += data
            first,last = frames.find(b'\x7D'),frames.find(b'\x7E')
            while first != -1 and last != -1:
                if last < first:
                    frames = frames[last + 1:]
                else:
                    # 防止和校验结果也出现0x7E
                    if len(frames)>last+1 and frames[last+1] == 0x7E:
                        last += 1
                    frame = frames[first:last + 1]
                    frames = frames[last + 1:]
                    self.frame_queue.put(frame)
                    count += 1
                    # self.send_frame(frame)
                    if int(time.time() - last_time) >= 2:
                        print("服务信息：平均帧率:%s/s, 运行时间:%ds,frame_len: %d" % (str(int(count / (time.time() - last_time))),int(time.time()-start),len(frames)))
                        last_time = time.time()
                        count = 0
                first,last = frames.find(b'\x7D'),frames.find(b'\x7E')

    def __exit__(self, exc_type, exc_value, traceback):
        self.server_thread.join()
        self.socket.close()


# queue_frame = Queue()
# server =  UdpServer(queue_frame,port=3333)
# while True:
#     if queue_frame.qsize()>0:
#         # print(queue_frame.get())
#         pass