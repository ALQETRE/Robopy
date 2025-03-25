#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port
from pybricks.tools import wait
import threading

import robopy as r


ev3 = EV3Brick()

left_motor = r.Motor(Port.A, wheel_diameter= 70)
right_motor = r.Motor(Port.B, wheel_diameter= 70)

bot = r.DriveBase(left_motor, right_motor, 120, ev3, optimal_battery_range= (7.9, 8.3))

print("Start")
bot.set_friction("driving")

# Code Segment

print("End")
wait(1000)