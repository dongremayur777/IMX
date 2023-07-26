import gi
import os

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

def update_temperature(overlay):
    temp_file = "/sys/class/thermal/thermal_zone0/temp"
    if os.path.exists(temp_file):
        with open(temp_file, 'r') as f:
            temperature = int(f.read()) / 1000.0
            text = f"Temperature: {temperature} C"
            overlay.set_property('text', text)
            overlay.set_property('shaded-background', False)

    # Reschedule the update function to run again in 1 second
    return True

pipeline = Gst.parse_launch(" v4l2src device=/dev/video0 ! textoverlay name=overlay ! vpuenc_h264 bitrate=1000 ! rtph264pay ! udpsink host=192.168.0.87 port=9001")

overlay = pipeline.get_by_name("overlay")

# schedule a periodic update of the temperature value
GLib.timeout_add_seconds(1, update_temperature, overlay)

pipeline.set_state(Gst.State.PLAYING)

# run the main loop
loop = GLib.MainLoop()
loop.run()

