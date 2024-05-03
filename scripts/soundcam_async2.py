#! /usr/bin/env python3

import time
import asyncio
from asyncio import StreamReader, StreamWriter

from multiprocessing import Process, Queue, Pipe
from soundcam_protocol import CommandCodes, DataMessages, Device, DataObjects, CameraProtocol
from soundcam_ros import SoundcamROS




FIRMWARE_VERSION = '2.8.3.0'
CONNECT_TIMEOUT = 25 #seconds

class SoundClientProtocol(asyncio.Protocol):
    def __init__(self, on_con_lost, conf:dict, q:Queue, debug:bool=True):
        print('Firmware Version: ', FIRMWARE_VERSION)
        self.debug = debug
        self.testing = False
        self.invokeId = 1
        self.cfg = conf
        self.globalQ = q
        
        self.protocol = CameraProtocol(protocol=self.cfg['protocol'], debug=self.debug)
        self.on_con_lost = on_con_lost

        self.message = b'\x84\x01\x00\x00\x04\x00\x00\x00\x03\x00\x00\x00' #camera id

    def connection_made(self, transport):
        transport.write(self.message)
        print('Data sent: {!r}'.format(self.message))
        time.sleep(1.0)
        transport.write(self.message)

    def data_received(self, data):
        print('Data received data: ', data)

    def connection_lost(self, exc):
        print('The server closed the connection')
        self.on_con_lost.set_result(True)

async def main():
    while(True):
        print('Running ...')
        await asyncio.sleep(1.0)

async def start():
    # Get a reference to the event loop as we plan to use
    # low-level APIs.
    loop = asyncio.get_running_loop()

    on_con_lost = loop.create_future()
    task1 = asyncio.create_task(main())

    ip = '169.254.169.138'
    port = 6340

    pubObj = SoundcamROS()
    globalQ = Queue()
    transport, protocol = await loop.create_connection(
        lambda: SoundClientProtocol(on_con_lost, pubObj.cfg, q=globalQ),
        ip, port)
    

    # Wait until the protocol signals that the connection
    # is lost and close the transport.
    try:
        await on_con_lost
    finally:
        transport.close()
    


if __name__ == '__main__':

    asyncio.run(start())
    
