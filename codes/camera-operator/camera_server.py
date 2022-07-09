import serial
import time
import sys
import threading
import tty, termios

# commands from server
COMM_START_RECORDING = 254
COMM_STOP_RECORDING = 255
COMM_SYNC_LOW = 200
COMM_SYNC_HIGH = 250

#info to server
COMM_STATE_IDLE = 0
COMM_STATE_RECORDING = 16


class bcolors:
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

user_command = 0

def getch():
  old_settings = termios.tcgetattr(0)
  new_settings = old_settings[:]
  new_settings[3] &= ~termios.ICANON
  try:
    termios.tcsetattr(0, termios.TCSANOW, new_settings)
    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(0, termios.TCSANOW, old_settings)
  return ch

key_pressed = ""

def listen_keys():
    global key_pressed
    while True:
        key_pressed = getch()

incoming_msgs = []




if(len(sys.argv)<3):
    print("please provide the baudrate and serial port number")
    exit()

s = serial.Serial('/dev/ttyS{}'.format(int(sys.argv[2])),int(sys.argv[1]),parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, timeout = 1, write_timeout = 1)

print('using /dev/ttyS{}'.format(int(sys.argv[2])))


start = time.time()

threading.Thread(target=listen_keys).start()

sync_msg = COMM_SYNC_LOW

recording = False

while(True):
    cameras_states = [-1]*16

    msg = s.read(16)
    for i in range(len(msg)):
        print(msg[i])
        if msg[i] < COMM_STATE_RECORDING and msg[0] >= COMM_STATE_IDLE:
            camid = int(msg[i])
            cameras_states[camid] = 0
        if msg[i] < COMM_STATE_RECORDING+16 and msg[0] >= COMM_STATE_RECORDING:
            camid = int(msg[i]-16)
            cameras_states[camid] = 1

    curr_time = time.time()
    if curr_time-start>2:
        status_string = ""
        # if recording:
        #     s.write(bytes([sync_msg]))
        #     s.flush()
        #     sync_msg +=1
        #     if sync_msg == 255:
        #         sync_msg = COMM_SYNC_LOW
        for i in range(len(cameras_states)):
            if cameras_states[i] == 0:
                status_string += "camera {:d} is".format(i) + bcolors.OKBLUE + " IDLE " + bcolors.ENDC
            if cameras_states[i] == 1:
                status_string += "camera {:d} is".format(i) + bcolors.OKGREEN + " RECORDING " + bcolors.ENDC
        start = curr_time
        print(status_string)
        

    if key_pressed == "r":
        s.write(bytes([COMM_START_RECORDING]))
        print("Sending start recording")
        key_pressed = ""
        recording = True
    if key_pressed == "s":
        s.write(bytes([COMM_STOP_RECORDING]))
        print("Sending stop recording")
        key_pressed = ""
        recording = False
