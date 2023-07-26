import gi
import os
import serial
import socket

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

def update_text_overlay(overlay):
    # check if there is any data available at UART
    if uart.in_waiting > 0:
        # read the data from UART
        data = uart.readline().decode('utf-8').rstrip()
        # set the overlay text to the received data
        overlay.set_property('text', data)
        overlay.set_property('shaded-background', False)

        # send the data to a UDP socket
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.sendto(data.encode('utf-8'), ('192.168.0.87', 9002))
        udp_socket.close()

    # Reschedule the update function to run again in 1 second
    return True

# configure serial communication
uart = serial.Serial('/dev/ttymxc3', 9600)

pipeline = Gst.parse_launch(" v4l2src device=/dev/video0 ! textoverlay name=overlay ! vpuenc_h264 ! rtph264pay ! udpsink host=192.168.0.87 port=9001")

overlay = pipeline.get_by_name("overlay")

# schedule a periodic update of the overlay text
GLib.timeout_add_seconds(1, update_text_overlay, overlay)

pipeline.set_state(Gst.State.PLAYING)

# run the main loop
loop = GLib.MainLoop()
loop.run()

uart.close()

