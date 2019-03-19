from contextlib import ExitStack

import serial

# cmds = [
#     'usbsd'
# ]

cmds = [
    'setmapping2 YUYV 640 480 30.0 WaltonRobotics DeepSpace',
    'setpar serout Hard',
    'setpar serlog USB',
    'streamon'
]

with ExitStack() as stack:

    serusb = stack.enter_context(serial.Serial('/dev/ttyACM0', 115200))
    serout = stack.enter_context(serial.Serial('/dev/ttyUSB1', 115200))

    for cmd in cmds:
        serout.write(f"{cmd:s}\n".encode("ASCII"))
    while True:
        if serusb.in_waiting > 0:
            print(f"log: {serusb.readline()!s}")
        if serout.in_waiting > 0:
            print(f"out: {serout.readline()!s}")
