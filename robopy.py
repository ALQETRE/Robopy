#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
import pybricks.ev3devices as dev # Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.media.ev3dev import Font

import math
from time import sleep
from json import load, dump

# TODO: Check the motor backlash after holding them
# TODO: Test ratios

Drive = 0

Right = 1
Left = -1

big_font = Font(size= 20, bold= True)

def wait_for_button(ev3):
    while not ev3.buttons.pressed():
        wait(10)
    while ev3.buttons.pressed():
        wait(10)

def black_lf_eval(target, sensor):
    reflection = sensor.reflection()
    error = target - reflection
    return error


class Motor:
    def __init__(self, motor_port, motor_type= Drive, wheel_diameter= 0, ratio= 1):
        self.motor = dev.Motor(motor_port)
        self.type = motor_type
        self.log_mode = True
        self.total_dist = 0

        if self.type == Drive:
            self._init_drive_motor(wheel_diameter, ratio)

    def _init_drive_motor(self, wheel_diameter, ratio):
        self.friction = 1
        self.diameter = wheel_diameter
        self.circ = math.pi * self.diameter
        self.ratio = ratio

    def run(self, dist, speed, wait= False):
        angle = dist / self.circ * 360
        angle /= self.ratio
        angle *= self.friction

        if self.log_mode:
            self.total_dist += dist

        self.motor.run_angle(speed, angle, wait= wait, then= Stop.HOLD)

    def run_free(self, speed):
        self.motor.run(speed)

    def free(self):
        self.motor.reset_angle(0)
        self.motor.stop()

    def lock(self):
        self.motor.hold()

    def reset_angle(self):
        self.motor.reset_angle(0)

    def get_movement(self):
        return self.motor.angle() * self.circ / 360 * self.ratio / self.friction

class DriveBase:
    def __init__(self, left_motor : Motor, right_motor : Motor, axle_len : int, ev3, optimal_battery_range= (0, 9)):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.axle_len = axle_len

        self.left_motor.lock()
        self.left_motor.reset_angle()
        self.right_motor.lock()
        self.right_motor.reset_angle()

        self.ev3 = ev3
        self.battery_range = optimal_battery_range

        self.battery_check(show= True)

        self.frictions = {"normal": [1, 1]}
        with open("frictions.json", encoding= "utf-8", mode= "r") as file:
            self.frictions = load(file)

        self.log_queue = []

    def battery_check(self, show= False):
        self.battery_voltage = self.ev3.battery.voltage()
        self.battery_voltage = self.battery_voltage / (10**(len(str(self.battery_voltage))-1))
        show = False
        if self.battery_voltage < self.battery_range[0]:
            print("Battery under range!")
            self.ev3.speaker.beep(frequency=500, duration=300)
            wait(100)
            self.ev3.speaker.beep(frequency=500, duration=300)
            show = True
        elif self.battery_voltage > self.battery_range[1]:
            print("Battery above range!")
            self.ev3.speaker.beep(frequency=600, duration=300)
            wait(100)
            self.ev3.speaker.beep(frequency=600, duration=300)
            show = True     

        if show:
            print("Battery voltage: " + str(self.battery_voltage) + "V")

    # Line following:
    def follow_line(self, speed, total_dist, line_follower= None):
        prev_line_follower = None
        if not line_follower is None:
            prev_line_follower = self.line_follower
            self.line_follower = line_follower


        total_angle = total_dist / self.left_motor.circ * 360
        angle_travelled = 0

        sum_of_errors = 0
        prev_error = 0

        prev_left_angle = self.left_motor.motor.angle()
        prev_right_angle = self.right_motor.motor.angle()

        while total_angle >= angle_travelled:
            error = self.line_follower.eval_function(self.line_follower.target, self.line_follower.sensor) * self.line_follower.side
            sum_of_errors += error

            if not self.line_follower.Kp is None:
                propotional = error * self.line_follower.Kp
            else:
                propotional = 0

            if not self.line_follower.Ki is None:
                integral = sum_of_errors * self.line_follower.Ki
            else:
                integral = 0

            if not self.line_follower.Kd is None:
                derivate = (error - prev_error) * self.line_follower.Kd
            else:
                derivate = 0

            prev_error = error

            correction = propotional + integral + derivate

            left_speed = speed + correction
            right_speed = speed - correction

            self.left_motor.run_free(left_speed)
            self.right_motor.run_free(right_speed)

            self.left_motor.run_free(left_speed)
            self.right_motor.run_free(right_speed)

            wait(5)
            angle_travelled = ((self.left_motor.motor.angle() - prev_left_angle) + (self.right_motor.motor.angle() - prev_right_angle)) / 2

        left_dist_travelled = (self.left_motor.motor.angle() - prev_left_angle) / 360 * self.left_motor.circ
        right_dist_travelled = (self.right_motor.motor.angle() - prev_right_angle) / 360 * self.right_motor.circ

        self.left_motor.total_dist += left_dist_travelled
        self.right_motor.total_dist += right_dist_travelled

        self.left_motor.lock()
        self.right_motor.lock()

        if not line_follower is None:
            self.line_follower = prev_line_follower

    def align(self, repeats, speed):
        sum_of_errors = 0
        prev_error = 0

        for i in range(repeats):
            error = self.eval_function(self.target, self.sensor) * self.side
            sum_of_errors += error

            propotional = error * self.Kp
            integral = sum_of_errors * self.Ki
            derivate = (error - prev_error) * self.Kd

            prev_error = error

            correction = propotional + integral + derivate

            left_speed = correction + speed
            right_speed = -correction - speed

            self.left_motor.run_free(left_speed)
            self.right_motor.run_free(right_speed)

            wait(10)

    def set_follow_line(self, line_follower : LineFollower):
        self.line_follower = line_follower

    # Movement:
    def grid_move(self, speed, move_vector= None, max_turn_rad= -1, rotation= 0):
        if move_vector:
            front_dist = move_vector[0]
            side_dist = move_vector[1]

            diff = abs(front_dist) - abs(side_dist)
            if diff == 0: # They are the same
                front_add_dist = 0
                side_add_dist = 0

                rad = abs(front_dist)
            elif diff > 0: # Front is bigger
                front_add_dist = diff
                side_add_dist = 0

                rad = abs(side_dist)
            elif diff < 0: #Side is bigger
                side_add_dist = diff
                front_add_dist = 0

                rad = abs(front_dist)

            if max_turn_rad == -1:
                max_turn_rad = rad

            if rad > max_turn_rad:
                rad_diff = rad - max_turn_rad
                rad = max_turn_rad
                front_add_dist += rad_diff
                side_add_dist += rad_diff

            if rad != 0:
                dist = rad * 2 * math.pi / 4

                small_rad = rad - (self.axle_len / 2)
                small_dist = small_rad * 2 * math.pi / 4

                big_rad = rad + (self.axle_len / 2)
                big_dist = big_rad * 2 * math.pi / 4

                time = dist / speed
                small_speed = small_dist / time
                big_speed = big_dist / time

            if front_dist > 0:
                # Pre curve:
                if front_add_dist != 0:
                    self.right_motor.run(front_add_dist, speed, wait= False)
                    self.left_motor.run(front_add_dist, speed, wait= True)

                if rad != 0:
                    # Curve:
                    if side_dist > 0:
                        self.left_motor.run(big_dist, big_speed, wait= False)
                        self.right_motor.run(small_dist, small_speed, wait= True)
                    if side_dist < 0:
                        self.right_motor.run(big_dist, big_speed, wait= False)
                        self.left_motor.run(small_dist, small_speed, wait= True)

                # Post curve:
                if side_add_dist != 0:
                    self.right_motor.run(side_add_dist, speed, wait= False)
                    self.left_motor.run(side_add_dist, speed, wait= True)

            elif front_dist < 0:
                # Pre curve:
                if front_add_dist != 0:
                    self.right_motor.run(-front_add_dist, speed, wait= False)
                    self.left_motor.run(-front_add_dist, speed, wait= True)
                
                if rad != 0:
                    # Curve:
                    if side_dist > 0:
                        self.left_motor.run(-big_dist, big_speed, wait= False)
                        self.right_motor.run(-small_dist, small_speed, wait= True)
                    if side_dist < 0:
                        self.right_motor.run(-big_dist, big_speed, wait= False)
                        self.left_motor.run(-small_dist, small_speed, wait= True)

                # Post curve:
                if side_add_dist != 0:
                    self.right_motor.run(side_add_dist, speed, wait= False)
                    self.left_motor.run(side_add_dist, speed, wait= True)

        if rotation != 0:
            rot_dist = self.axle_len * math.pi
            rot_dist = rot_dist / 360 * rotation
            self.right_motor.run(-rot_dist, speed, wait= False)
            self.left_motor.run(rot_dist, speed, wait= True)

    def move(self, speed, dist):
        self.left_motor.run(dist, speed, wait= False)
        self.right_motor.run(dist, speed, wait= True)

    def turn(self, speed, radius= 0, angle= 0):
        true_angle = angle
        angle = abs(angle)

        dist = radius * 2 * math.pi / 360 * angle

        small_rad = radius - (self.axle_len / 2)
        small_dist = small_rad * 2 * math.pi / 360 * angle

        big_rad = radius + (self.axle_len / 2)
        big_dist = big_rad * 2 * math.pi / 360 * angle

        time = dist / speed
        if time == 0:
            small_speed = speed
            big_speed = speed
        else:
            small_speed = small_dist / time
            big_speed = big_dist / time

        if true_angle > 0:
            self.right_motor.run(small_dist, small_speed, wait= False)
            self.left_motor.run(big_dist, big_speed, wait= True)
        else:
            self.right_motor.run(big_dist, big_speed, wait= False)
            self.left_motor.run(small_dist, small_speed, wait= True)

        self.left_motor.lock()
        self.right_motor.lock()
    
    # Calibration:
    def calibrate_friction(self, move_func, friction : str):
        
        
        self.left_motor.log_mode = True
        self.left_motor.total_dist = 0
        self.left_motor.reset_angle()

        self.right_motor.log_mode = True
        self.right_motor.total_dist = 0
        self.right_motor.reset_angle()


        move_func(self)

        left_total_dist = self.left_motor.total_dist
        right_total_dist = self.right_motor.total_dist

        screen = self.ev3.screen

        previous_left_friction = self.left_motor.friction
        previous_right_friction = self.right_motor.friction

        wait(1000)

        screen.clear()
        screen.print("\n")
        screen.set_font(big_font)
        screen.print("Correct the robot")

        self.left_motor.free()
        self.right_motor.free()

        wait_for_button(self.ev3)

        screen.clear()

        left_dist = self.left_motor.get_movement()
        right_dist = self.right_motor.get_movement()

        self.left_motor.lock()
        self.right_motor.lock()

        self.left_motor.friction /= left_total_dist / (left_total_dist + left_dist)
        self.right_motor.friction /= right_total_dist / (right_total_dist + right_dist)

        left_friction = round(self.left_motor.friction, 4)
        right_friction = round(self.right_motor.friction, 4)

        screen.print("\n")
        screen.set_font(big_font)

        screen.print("Left over: " + str(left_dist*previous_left_friction))
        print("Left over: " + str(left_dist*previous_left_friction))
        screen.print("Right over: " + str(right_dist*previous_right_friction))
        print("Right over: " + str(right_dist*previous_right_friction))

        screen.print()
        print()

        screen.print("Left friction: " + str(self.left_motor.friction))
        print("Left friction: " + str(self.left_motor.friction))
        screen.print("Right friction: " + str(self.right_motor.friction))
        print("Right friction: " + str(self.right_motor.friction))

        self.left_motor.reset_angle()
        self.right_motor.reset_angle()

        wait_for_button(self.ev3)
        screen.clear()

        self.frictions[friction] = [self.left_motor.friction, self.right_motor.friction]

        with open("frictions.json", encoding= "utf-8", mode= "w") as file:
            dump(self.frictions, file)

    def set_friction(self, friction= "normal"):
        self.left_motor.friction = self.frictions[friction][0]
        self.right_motor.friction = self.frictions[friction][1]

    # Run until:
    def follow_line_until(self, speed, until_func, line_follower= None, args= None):
        prev_line_follower = None
        if not line_follower is None:
            prev_line_follower = self.line_follower
            self.line_follower = line_follower

        angle_travelled = 0

        sum_of_errors = 0
        prev_error = 0

        prev_left_angle = self.left_motor.motor.angle()
        prev_right_angle = self.right_motor.motor.angle()

        while not until_func(*args):
            error = self.line_follower.eval_function(self.line_follower.target, self.line_follower.sensor) * self.line_follower.side
            sum_of_errors += error

            if not self.line_follower.Kp is None:
                propotional = error * self.line_follower.Kp
            else:
                propotional = 0

            if not self.line_follower.Ki is None:
                integral = sum_of_errors * self.line_follower.Ki
            else:
                integral = 0

            if not self.line_follower.Kd is None:
                derivate = (error - prev_error) * self.line_follower.Kd
            else:
                derivate = 0

            prev_error = error

            correction = propotional + integral + derivate

            left_speed = speed + correction
            right_speed = speed - correction

            self.left_motor.run_free(left_speed)
            self.right_motor.run_free(right_speed)

            angle_travelled = ((self.left_motor.motor.angle() - prev_left_angle) + (self.right_motor.motor.angle() - prev_right_angle)) / 2

        left_dist_travelled = (self.left_motor.motor.angle() - prev_left_angle) / 360 * self.left_motor.circ
        right_dist_travelled = (self.right_motor.motor.angle() - prev_right_angle) / 360 * self.right_motor.circ

        self.left_motor.total_dist += left_dist_travelled
        self.right_motor.total_dist += right_dist_travelled

        self.left_motor.lock()
        self.right_motor.lock()

        if not line_follower is None:
            self.line_follower = prev_line_follower

        #return angle_travelled

    def move_until(self, speed, until_func, args= None):
        prev_left_angle = self.left_motor.motor.angle()
        prev_right_angle = self.right_motor.motor.angle()

        while not until_func(*args):
            self.left_motor.run_free(speed)
            self.right_motor.run_free(speed)

        left_dist_travelled = (self.left_motor.motor.angle() - prev_left_angle) / 360 * self.left_motor.circ
        right_dist_travelled = (self.right_motor.motor.angle() - prev_right_angle) / 360 * self.right_motor.circ
        self.left_motor.total_dist += left_dist_travelled
        self.right_motor.total_dist += right_dist_travelled

        self.left_motor.lock()
        self.right_motor.lock()

    def turn_until(self, speed, until_func, radius= 0, direction= Right, args= None):
        true_angle = direction
        angle = 90

        dist = radius * 2 * math.pi / 360 * angle

        small_rad = radius - (self.axle_len / 2)
        small_dist = small_rad * 2 * math.pi / 360 * angle

        big_rad = radius + (self.axle_len / 2)
        big_dist = big_rad * 2 * math.pi / 360 * angle

        time = dist / speed
        if time == 0:
            small_speed = speed
            big_speed = speed
        else:
            small_speed = small_dist / time
            big_speed = big_dist / time

        prev_left_angle = self.left_motor.motor.angle()
        prev_right_angle = self.right_motor.motor.angle()

        while not until_func(*args):
            if direction > 0:
                self.right_motor.run_free(small_speed)
                self.left_motor.run_free(big_speed)
            else:
                self.right_motor.run_free(big_speed)
                self.left_motor.run_free(small_speed)

        left_dist_travelled = (self.left_motor.motor.angle() - prev_left_angle) / 360 * self.left_motor.circ
        right_dist_travelled = (self.right_motor.motor.angle() - prev_right_angle) / 360 * self.right_motor.circ
        self.left_motor.total_dist += left_dist_travelled
        self.right_motor.total_dist += right_dist_travelled

        self.left_motor.lock()
        self.right_motor.lock()


class LineFollower:
    def __init__(self, eval_function, sensor, target, side= Right, Kp= None, Ki= None, Kd= None):
        self.eval_function = eval_function
        self.sensor = sensor
        self.target = target
        self.side = side
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd