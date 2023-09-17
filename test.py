# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time

from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

def sleep(servos):
    for i in range(4):
        if i % 2 == 0:
            servos[i*2].angle = 180
        else:
            servos[i*2].angle = 0
        servos[i*2+1].angle = 160

def wakeup(servos):
    for j in range(45):
        for i in range(4):
            if i % 2 == 0:
                servos[i*2].angle -= 1
            else:
                servos[i*2].angle += 1
            servos[i*2+1].angle -= 3

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50
# pca.reset()
# raise Exception("stop")
servo_idx = [0, 1, 2, 3, 4, 5, 6, 7]
angle = [150, 60, 30, 120, 150, 60, 30, 120]
servos = [servo.Servo(pca.channels[i], min_pulse=480, max_pulse=2600) for i in servo_idx]
direction = False

# sleep(servos)
# wakeup(servos)
# print(servos[0].angle)

for i in range(len(servos)):
    servos[i].angle = angle[i]
    print(servos[i].angle)


# for i in range(4):
#     if i % 2 == 0:
#         servos[i*2].angle = 135
#         servos[i*2+1].angle = 50
#     else:
#         servos[i*2].angle = 45
#         servos[i*2+1].angle = 130
#     # servos[i*2+1].angle = 30
# time.sleep(1)

# for j in range(300):
#     for i in range(4):
#         if i % 2 == 0:
#             servos[i*2].angle = 135
#         else:
#             servos[i*2].angle = 45
#         if direction:
#             servos[i*2+1].angle += 2
#         else:
#             servos[i*2+1].angle -= 2
#         print(servos[i*2+1].angle)

#     if servos[1].angle >= 70:
#         direction = False
#     elif servos[1].angle <= 30:
#         direction = True
#     time.sleep(0.005)

pca.deinit()