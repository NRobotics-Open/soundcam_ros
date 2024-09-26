#!/usr/bin/env python3

# import socket
# import dearpygui.dearpygui as dpg

# dpg.create_context()
# dpg.create_viewport(title='Custom Title', width=600, height=200)
# dpg.setup_dearpygui()

# with dpg.window(label="Example Window"):
#     dpg.add_text("Hello, world")

# dpg.show_viewport()

# #Socket setup
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.listen(1)

# # below replaces, start_dearpygui()
# while dpg.is_dearpygui_running():
#     # insert here any code you would like to run in the render loop
#     # you can manually stop by using stop_dearpygui()
#     print("this will run every frame")
#     dpg.render_dearpygui_frame()

# dpg.destroy_context()

import socket

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind(('localhost', 65432))

# Listen for incoming connections
server_socket.listen(1)

print('Waiting for a connection...')
connection, client_address = server_socket.accept()

try:
    while True:
        data = connection.recv(1024)
        if data:
            print('Received:', data.decode('utf-8'))
        else:
            break
finally:
    connection.close()
