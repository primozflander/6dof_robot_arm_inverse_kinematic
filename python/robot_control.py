# installing bluetooth -> pip install PyBluez==0.22 --user

from tkinter import *
import bluetooth
import time

# print("performing inquiry...")
# nearby_devices = bluetooth.discover_devices(lookup_names=True)
# print("found %d devices" % len(nearby_devices))
# for name, addr in nearby_devices:
#     print(" %s - %s" % (addr, name))


# def get_device_name(device_name):
#     device_address = None
#     nearby_devices = bluetooth.discover_devices()
#     for device in nearby_devices:
#         print(device)
#         print(bluetooth.lookup_name(device))
#         if device_name == bluetooth.lookup_name(device):
#             device_address = device
#             break
#     if device_address is not None:
#         print("Found esp32 device with address", device_address)
#     else:
#         print("Could not find esp32 device nearby")
#     return device_address


def move_to_position(e):
    angles = str(w1.get()) + "," + str(w2.get()) + "," + str(w3.get()) + "," + str(w4.get()) + "," + str(w5.get()) + "," + str(w6.get()) + "," + str(100 - w7.get())
    print(angles)
    socket.send(angles)


def reset_position():
    angles = "90,20,160,90,90,90"
    print(angles)
    socket.send(angles)


print("Establishing connection...")
#esp_address = get_device_name("ESP32RobotArm6Dof")
esp_address = "30:AE:A4:38:85:9A"
socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
try:
    socket.connect((esp_address, 1))
except:
    print("Connection cannot be established, make sure that the device is powered on and try again")
    time.sleep(2)
    exit()
print("Ready")

master = Tk()
w1 = Scale(master, from_=0, to=180, length=600, tickinterval=10, orient=HORIZONTAL, label="Angle 1")
w1.bind("<ButtonRelease-1>", move_to_position)
w1.set(90)
w1.pack()

w2 = Scale(master, from_=20, to=160, length=600, tickinterval=10, orient=HORIZONTAL, label="Angle 2")
w2.bind("<ButtonRelease-1>", move_to_position)
w2.set(20)
w2.pack()

w3 = Scale(master, from_=40, to=170, length=600, tickinterval=10, orient=HORIZONTAL, label="Angle 3")
w3.bind("<ButtonRelease-1>", move_to_position)
w3.set(160)
w3.pack()

w4 = Scale(master, from_=0, to=180, length=600, tickinterval=10, orient=HORIZONTAL, label="Angle 4")
w4.bind("<ButtonRelease-1>", move_to_position)
w4.set(90)
w4.pack()

w5 = Scale(master, from_=0, to=180, length=600, tickinterval=10, orient=HORIZONTAL, label="Angle 5")
w5.bind("<ButtonRelease-1>", move_to_position)
w5.set(90)
w5.pack()

w6 = Scale(master, from_=0, to=180, length=600, tickinterval=10, orient=HORIZONTAL, label="Angle 6")
w6.bind("<ButtonRelease-1>", move_to_position)
w6.set(90)
w6.pack()

w7 = Scale(master, from_=1, to=100, length=600, tickinterval=10, orient=HORIZONTAL, label="Speed")
w7.set(75)
w7.pack()

Button(master, text='Reset', command=reset_position).pack()

mainloop()

socket.close()
