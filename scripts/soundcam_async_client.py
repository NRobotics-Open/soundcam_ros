#! /usr/bin/env python3

import time
import asyncio
from asyncio import StreamReader, StreamWriter

from multiprocessing import Process, Queue, Pipe
from soundcam_protocol import CameraProtocol

class SoundcamClientProtocol(asyncio.Protocol):
    def __init__(self, on_con_lost, query:bytes, q:Queue, debug:bool=True):
        self.globalQ = q
        self
        self.on_con_lost = on_con_lost
        self.message = query #camera id

    def connection_made(self, transport):
        print('Identifying device ...')
        transport.write(self.message)
        print('Data sent: {!r}'.format(self.message))

    def data_received(self, data):
        print('Data received data: ', len(data))
        if(data):
            self.globalQ.put(data)

    def connection_lost(self, exc):
        print('The server closed the connection')
        self.on_con_lost.set_result(True)