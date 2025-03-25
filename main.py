#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port
from pybricks.tools import wait

import threading


import robopy as r


ev3 = EV3Brick()

color_sensor = ColorSensor(Port.S1)
dist_sensor = UltrasonicSensor(Port.S2)

left_motor = r.Motor(Port.A, wheel_diameter= 70)
right_motor = r.Motor(Port.B, wheel_diameter= 70)

bot = r.DriveBase(left_motor, right_motor, 120, ev3, optimal_battery_range= (7.9, 8.3))

black_line_follower = r.LineFollower(r.black_lf_eval, color_sensor, 60, r.Right, 0.9, 0, 5)

def until_wall(dist):
    current = dist_sensor.distance()
    return current < dist

def calibrate_move1(bot):
    bot.turn(200, angle= 180)

def calibrate_move2(bot):
    bot.move(350, 280*3)

print("Start")

bot.set_friction("driving")
#bot.calibrate_friction(calibrate_move1, "driving")

bot.set_follow_line(black_line_follower)

# bot.follow_line(350, 280*2+140)
# bot.follow_line_until(150, until_wall)

bot.follow_line(300, 280)
bot.turn(150, angle= 90)
bot.move(300, 280*2+140)
bot.turn(200, radius= 140, angle= -90)
bot.follow_line_until(250, until_wall, args= [190])
bot.turn(200, radius=140, angle= 90)
bot.move_until(150, until_wall, args= [30])
bot.turn(150, angle= 90)
bot.move_until(200, until_wall, args= [40])

print("End")
wait(1000)