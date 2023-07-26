import gi
import os
import serial
import socket
from multiprocessing import Process

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)


def text(uart):
    def update_text_overlay(overlay, uart):
        if uart.in_waiting > 0:
            data = uart.readline().decode('latin1',errors='ignore')
            overlay.set_property('text', data)
            overlay.set_property('font_desc','Sans 15')
            overlay.set_property('shaded-background', False)
            data = uart.readline().decode('utf-8').rstrip()
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.sendto(data.encode('utf-8'), ('192.168.0.87', 9002))
            udp_socket.close()

        return True

    pipeline = Gst.parse_launch("v4l2src device=/dev/video0 ! textoverlay name=overlay ! vpuenc_h264 ! rtph264pay ! udpsink host=192.168.0.87 port=9001 sync=false")
    overlay = pipeline.get_by_name("overlay")
    GLib.timeout_add(0.5, update_text_overlay, overlay, uart)
    pipeline.set_state(Gst.State.PLAYING)
    loop = GLib.MainLoop()
    loop.run()
    uart.close()

if __name__ == '__main__':
    uart = serial.Serial('/dev/ttymxc3', 9600)
   # q = Process(target=uart1, args=())
   # q.start()
    p = Process(target=text, args=(uart,))
    p.start()
   # q.join()
    p.join()

