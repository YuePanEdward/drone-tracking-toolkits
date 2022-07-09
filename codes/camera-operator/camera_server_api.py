import PySimpleGUI as sg
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.io import IOLine, IOMode, IOValue
import time
import io
from PIL import Image
import math

# image = Image.open('test.jpg')
# image.thumbnail((200,200))
# bio = io.BytesIO()
# image.save(bio, format="PNG")

sg.theme('DarkAmber') 

col1 = [[sg.Button('Device discovery')],
        [sg.Button('Get status of cameras')],
        [sg.Button('Start recording')],
        [sg.Button('Stop recording')],
        [sg.Button('Get preview')],
            [sg.Text('Discovered devices:')],
            [sg.Listbox(values=[], size=(30, 12), key='-LIST-', enable_events=True)],
            [sg.Text('Status:', size=(50, 1), key ='-STATUS-')]]

col2 = [[sg.Image(key="-IMAGE-%d-%d-" % (i,j)) for i in range(5)] for j in range(2)]

layout = [[ sg.Column(col1, element_justification = 'l')  , 
            sg.Column(col2, element_justification = 'l')
            ]]

window = sg.Window('Camera operator server', layout, size=(1500, 500))

device = XBeeDevice("/dev/ttyUSB0", 115200*8, _sync_ops_timeout=99999999)

device.open()

xnet = device.get_network()

devices = {}

buffer = io.BytesIO()
receiving_image = False
packet_n = 0
def data_received_callback(xbee_message):
    global window
    global buffer
    global receiving_image
    global packet_n
    global devices
    address = xbee_message.remote_device.get_64bit_addr()
    command = 0
    try:
        data = xbee_message.data.decode("utf8")
        print("Received data from %s: %s" % (address, data))
        if data == "RECORDING":
            devices[xbee_message.remote_device.__str__()][2] = "RECORDING"
            # window['-LIST-'].update([node + " : " + status for node, status in devices.items()])
            command = 1
        if data == "IDLE":
            devices[xbee_message.remote_device.__str__()][2] = "IDLE"
            # window['-LIST-'].update([node + " : " + status for node, status in devices.items()])
            command = 1
        if data == "PREVIEW_START":
            buffer =   io.BytesIO()
            receiving_image = True
            command = 1
        if data == "PREVIEW_END":
            command = 1
            # with open("test_inc.jpg", "wb") as outfile:
                # Copy the BytesIO stream to the output file
                # outfile.write(buffer.getbuffer())
            idx = devices[xbee_message.remote_device.__str__()][1]
            buffer.seek(0)
            img = Image.open(buffer)
            img.thumbnail((200,200))
            print("opened image from buffer")
            buffer2 = io.BytesIO()
            img.save(buffer2,format = "PNG")
            buffer2.seek(0)
            del img
            receiving_image = False
            packet_n = 0
            print(idx)
            # buffer.seek(0)
            j = math.floor(idx/5.0)
            i = idx % 5
            print(i)
            print(j)
            window['-IMAGE-%d-%d-' % (i,j)].update(data=buffer2.getvalue())
            print("DONE")
    except Exception as e:
        print(e)

    if receiving_image and not command:
        buffer.write(xbee_message.data)
        print("Received packet #%d " % packet_n)
        packet_n +=1    
    


# Add the callback.
device.add_data_received_callback(data_received_callback)



while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Cancel': # if user closes window or clicks cancel
        break
    try:
        if event == 'Device discovery':
            xnet.start_discovery_process(deep=True, n_deep_scans=1)
            while xnet.is_discovery_running():
                time.sleep(0.5)
            nodes = xnet.get_devices()
            window['-STATUS-'].update('Status: Device discovery finished')
            devices = {node.__str__() : [node, idx, ""] for node, idx in zip(nodes,range(len(nodes)))}
            window['-LIST-'].update([node for node, data in devices.items()])
        
        if event == 'Start recording':
            for node in nodes:
                device.send_data(node, "START")

        if event == 'Stop recording':
            for node in nodes:
                device.send_data(node, "STOP")

        if event == 'Get status of cameras':
            for node in nodes:
                device.send_data(node, "STATUS")
        if event == 'Get preview':
            selected_node = window['-LIST-'].get()
            if(len(selected_node)>0):
                print(selected_node[0])
                device.send_data(devices[selected_node[0]][0], "PREVIEW")
    except Exception as e:
        print(e)
