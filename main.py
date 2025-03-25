#!/usr/bin/env pybricks-micropython
from spike import hub
from spike import motor
from spike import color_sensor
from spike import time
from hub import port

import robopy as r


hub = hub()

left_motor = r.Motor(port.A, wheel_diameter= 70)
right_motor = r.Motor(portB, wheel_diameter= 70)

bot = r.DriveBase(left_motor, right_motor, 120, hub, optimal_battery_range= (7.9, 8.3))

print("Start")
bot.set_friction("driving")

# Code Segment

print("End")
time.sleep_ms(1000)