import serial
import sys
import time
import threading
import os
import datetime
import numpy as np
import io
import subprocess
import multiprocessing
import shlex
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.io import IOLine, IOMode, IOValue
import picamera
from PIL import Image

# STATES
STATE_IDLE = 0
STATE_RECORDING = 1


def find_last_video_id():
    id = 0 # first video
    for root, dirs, files in os.walk(record_dir_root):
        for dirname in sorted(dirs):
            id = int(dirname[-5:])
    return id

def make_video(video_dir):
    command = "ffmpeg -framerate 60 -i " + video_dir + "frame%010d.jpg -codec copy " + video_dir + "video.mkv"
    subprocess.call(command,shell=True)
    global status 
    status = STATE_IDLE

def record_video_ffmpeg(video_dir):
    command = "ffmpeg -f v4l2 -input_format h264 -video_size 1920x1080 -i /dev/video0 -vcodec copy {}/video.avi".format(video_dir)
    global p
    p = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

def wrap_video_mp4():
    video_id = find_last_video_id()
    print("wrapping video id {}".format(video_id))
    video_dir = record_dir_root + "video{0:05d}/".format(video_id)
    command = "MP4Box -add {}/video.h264 -fps 30 {}/video.mp4".format(video_dir, video_dir)
    global p
    p = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

def capture_frames():
    video_id = find_last_video_id()+1
    print("recording video id {}".format(video_id))
    curr_video_dir = record_dir_root + "video{0:05d}/".format(video_id)
    os.mkdir(curr_video_dir)
    print("directory created")
    global status 
    global recording
    status = STATE_RECORDING
    record_video_ffmpeg(curr_video_dir)
    
# Define callback.
def data_received_callback(xbee_message):
    address = xbee_message.remote_device.get_64bit_addr()
    data = xbee_message.data.decode("utf8")
    print("Received data from %s: %s" % (address, data))
    global status
    if data == "START" and status == STATE_IDLE:
        status = STATE_RECORDING
        capture_frames()
        device.send_data(xbee_message.remote_device,"RECORDING")
    if data == "STOP" and status == STATE_RECORDING:
        p.stdin.write('q'.encode("GBK"))
        p.communicate()
        #wrap_video_mp4()
        status = STATE_IDLE
        device.send_data(xbee_message.remote_device,"IDLE")
    if data == "STATUS":
        device.send_data(xbee_message.remote_device, "RECORDING" if status else "IDLE")
    if data == "PREVIEW":
        out = capture_preview()
        print("out buffer has %d bytes" % out.getbuffer().nbytes)
        device.send_data(xbee_message.remote_device, "PREVIEW_START")
        out.seek(0)
        chunk = out.read(250)
        packet_n = 0
        while chunk:
            try:
                device.send_data(xbee_message.remote_device, chunk)
                chunk = out.read(250)
                print("Sending chunk #%d" % packet_n)
                packet_n += 1
            except:
                pass
        device.send_data(xbee_message.remote_device, "PREVIEW_END")


        

def capture_preview():
    with picamera.PiCamera() as camera:
        output = io.BytesIO()
        camera.resolution = (320, 240)
        camera.start_preview()
        time.sleep(2)
        camera.capture(output, format='jpeg')
        output.seek(0)
        image = Image.open(output)
        output = io.BytesIO()
        image.save(output, "JPEG", quality = 20)
        print("Captured %d bytes of image data" % output.getbuffer().nbytes)
        return output


if(len(sys.argv)<3):
    print("please provide the baudrate and camera id")
    exit()

cam_id = int(sys.argv[2])
print(cam_id)

bitrate = int(sys.argv[1])

device = XBeeDevice("/dev/ttyUSB0", bitrate)

device.open()

device.add_data_received_callback(data_received_callback)

status = STATE_IDLE

recording = False

record_dir_root = "/home/pi/videos/"

frameid = 0

start = time.time()

sync_timestamps = []
video_timestamps = []

while True:
    time.sleep(1)
