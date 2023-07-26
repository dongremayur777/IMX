import gi
import os
import serial
import socket
from multiprocessing import Process
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)


def uart1():	
    while True:
        data = uart.readline().decode('utf-8').rstrip()
        # set the overlay text to the received data
        #overlay.set_property('text', data)
        #overlay.set_property('shaded-background', False)

        # send the data to a UDP socket
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.sendto(data.encode('utf-8'), ('192.168.0.87', 9002))
        udp_socket.close()

        # add a small delay to allow other functions to also read from UART
        time.sleep(0.1)


def text():
    

    # configure serial communication
    pipeline = Gst.parse_launch(" v4l2src device=/dev/video0 ! textoverlay name=overlay ! vpuenc_h264 ! rtph264pay ! udpsink host=192.168.0.87 port=9001")
    overlay = pipeline.get_by_name("overlay")

    # schedule a periodic update of the overlay text
    GLib.timeout_add_seconds(0.01, update_text_overlay, overlay)

    pipeline.set_state(Gst.State.PLAYING)

    # run the main loop
    loop = GLib.MainLoop()
    loop.run()
    uart.close()


if __name__ == '__main__':
    # configure the serial port to use
    uart = serial.Serial('/dev/ttymxc3', 9600)

    # start the overlay process in a separate process
    q = Process(target=uart1, args=())
    q.start()

    # start the text process in a separate process
    p = Process(target=text, args=())
    p.start()

    # wait for both processes to finish
    q.join()
    p.join()

