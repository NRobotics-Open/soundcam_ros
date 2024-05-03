#! /usr/bin/env python3

import time
import asyncio
from asyncio import StreamReader, StreamWriter
from soundcam_protocol import CommandCodes, DataMessages, Device, DataObjects, CameraProtocol

ip = '169.254.169.138'
port = 6340

class EchoClientProtocol(asyncio.Protocol):
    def __init__(self, camera:CameraProtocol, message, on_con_lost):
        self.message = message
        self.on_con_lost = on_con_lost

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
    # Get a reference to the event loop as we plan to use
    # low-level APIs.
    loop = asyncio.get_running_loop()

    on_con_lost = loop.create_future()
    message = b'\x84\x01\x00\x00\x04\x00\x00\x00\x03\x00\x00\x00' #camera id

    transport, protocol = await loop.create_connection(
        lambda: EchoClientProtocol(message, on_con_lost),
        ip, port)

    # Wait until the protocol signals that the connection
    # is lost and close the transport.
    try:
        await on_con_lost
    finally:
        transport.close()



if __name__ == '__main__':

    asyncio.run(main())
    while(True):
        time.sleep(0.5)
        print('Running async!')
    
