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
import serial


class UdpServer:
    def __init__(self,queue=Queue(),port=3333):
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
        print("Send to %s, frame: %s",self.addr,frame)
        while self.addr is None and not self.event.is_set():
            time.sleep(1)
            print("wait address...")
        self.socket.sendto(frame,self.addr)

    def run_server(self):
        print("server start....")
        frames = b''
        count = 0
        last_time = time.time()
        start =  time.time()
        while not self.event.is_set():
            data, self.addr = self.socket.recvfrom(220)
            if not data: continue
            frames += data
            first = frames.find(b'\xfa')
            while first != -1 and len(frames)>=22:
                frame = frames[first:first+22]
                frames = frames[first+22:]
                if len(frame)!=22 or frame[1] < 0xa0:
                    print("frame",frame)
                first = frames.find(b'\xfa')
                self.frame_queue.put(frame)
                count += 1

    def __exit__(self, exc_type, exc_value, traceback):
        self.server_thread.join()
        self.socket.close()



class SerialServer:
    def __init__(self,queue=Queue(),addr='/dev/ttyUSB0',baut=115200):
        self.event = threading.Event()
        self.frame_queue = queue
        self.addr = addr
        self.baut = baut
        self.connected = False
        self.__enter__()

    def __enter__(self):
        try:
            self.ser=serial.Serial(self.addr,self.baut,timeout=0)
            self.connected = True
        except socket.error as e:
            print('Open failed:{}'.format(e))
            self.connected = False
            raise
        print('Starting server on addr={} baut={}'.format(self.addr,self.baut))
        # 启动线程读取数据
        self.server_thread = threading.Thread(target=self.run_server,name='run_server')
        self.server_thread.start()
        return self

    def send_frame(self,frame):
        print("Send to %s, frame: %s",self.addr,frame)
        self.ser.write(frame)

    def run_server(self):
        print("server start....")
        frames = b''
        count = 0
        last_time = time.time()
        start =  time.time()
        while not self.event.is_set():
            data = self.ser.readall()
            if not data: continue
            frames += data
            first = frames.find(b'\xfa')
            while first != -1 and len(frames)>=22:
                frame = frames[first:first+22]
                frames = frames[first+22:]
                if len(frame)!=22 or frame[1] < 0xa0:
                    print("frame",frame)
                first = frames.find(b'\xfa')

                self.frame_queue.put(frame)
                count += 1
                if int(time.time() - last_time) >= 1:
                    print("服务信息：平均帧率:%s/s, 运行时间:%ds,frames_len: %d" % (str(int(count / (time.time() - last_time))),int(time.time()-start),len(frames)))
                    last_time = time.time()
                    count = 0

    def __exit__(self, exc_type, exc_value, traceback):
        self.server_thread.join()
        self.socket.close()



class TcpServer:
    def __init__(self, queue=Queue(),port=3333):
        self.port = port
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.frame_queue = queue
        self.event = threading.Event()
        self.conn = None
        self.__enter__()

    def __enter__(self):
        try:
            self.socket.bind(("0.0.0.0", self.port))
        except socket.error as e:
            print('Bind failed:{}'.format(e))
            raise
        self.socket.listen()
        print('Starting server on port={} ip={}'.format(self.port,"0.0.0.0"))
        # 启动线程接受连接
        self.server_thread = threading.Thread(target=self._accept,name='accept',daemon=True)
        self.server_thread.start()
        return self

    # 接收连接
    def _accept(self):
        while not self.event.is_set(): #多人连接
            conn,client = self.socket.accept()  #阻塞
            self.conn = conn
            print("收到客户端连接",conn,client)
            # self.clients[client] = client
            # 受到连接开个线程来接收数据
            threading.Thread(target=self._recv, args=(conn, client), name='recv',daemon=True).start()

    def send_frame(self,frame):
        print("Send to %s, frame: %s",self.conn,frame)
        while self.conn is None and not self.event.is_set():
            time.sleep(1)
            print("wait address...")
        self.conn.send(frame)


    def _recv(self,conn, client):
        print("server start....")
        frames = b''
        count = 0
        last_time = time.time()
        start =  time.time()  
        while not self.event.is_set():
            data = conn.recv(1024)
            # print("recv:",len(data))
            frames += data
            first = frames.find(b'\xfa')
            while first != -1 and len(frames)>=22:
                frame = frames[first:first+22]
                frames = frames[first+22:]
                if len(frame)!=22 or frame[1] < 0xa0:
                    print("frame",frame)
                first = frames.find(b'\xfa')

                self.frame_queue.put(frame)
                count += 1
                if int(time.time() - last_time) >= 4:
                    print("服务信息：平均帧率:%s/s, 运行时间:%ds,frames_len: %d" % (str(int(count / (time.time() - last_time))),int(time.time()-start),len(frames)))
                    last_time = time.time()
                    count = 0

    def __exit__(self, exc_type, exc_value, traceback):
        self.server_thread.join()
        self.socket.close()
