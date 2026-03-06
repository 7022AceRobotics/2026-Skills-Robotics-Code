#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=50, axle_track=185)

left_color = ColorSensor(Port.S1)
right_color = ColorSensor(Port.S2)
ultrasonic = UltrasonicSensor(Port.S3)
ultrasonic_wall = UltrasonicSensor(Port.S4)
HasTube = bool(False)
grabber = Motor(Port.C)     # Medium motor (grabber)
elevator = Motor(Port.B)   # Elevator motor





line = int(1)
# -----------------------------
# CONSTANTS
# -----------------------------


SPEED = 40
TURN_GAIN = 1.2

# -----------------------------
# BASIC FUNCTIONS
# -----------------------------
def is_black(sensor):
    return sensor.reflection() < 50

def intersection_detected():
    return is_black(left_color) and is_black(right_color)

def F_intersection_detected():
    return is_black(right_color)

def follow_line():
    error = left_color.reflection() - right_color.reflection()
    robot.drive(SPEED, error * TURN_GAIN)
    print("error", error)

def stop_and_brake():
    robot.stop()
    left_motor.brake()
    right_motor.brake()
    wait(400)

def follow_line_for_seconds(seconds):
    elapsed = 0
    while elapsed < seconds * 1000:
        follow_line()
        wait(20)
        elapsed += 20

def turn_right(angle):
    stop_and_brake()
    robot.turn(angle)
    wait(400)

def turn_left(angle):
    stop_and_brake()
    robot.turn(-angle)
    wait(400)

# -----------------------------
# LINE FOLLOW HELPERS
# -----------------------------
def wait_after_intersection(seconds):
    elapsed = 0
    while elapsed < seconds * 1000:
        follow_line()
        wait(20)
        elapsed += 20

def follow_until_intersection():
    while True:
        follow_line()
        wait(20)
        if intersection_detected():
            stop_and_brake()
            break
def follow_until_F_intersection():
    while True:
        follow_line()
        wait(20)
        if F_intersection_detected():
            stop_and_brake()
            break
def follow_until_End():
    global HasTube
    while True:
        follow_line()
        wait(20)
        distance_cm = ultrasonic.distance() / 10
        ev3.screen.clear()
        ev3.screen.print("Distance: {:.1f} cm".format(distance_cm))
        if distance_cm < 25 and distance_cm > 20:
            stop_and_brake()
            ev3.screen.print("Has no Tube")
            break
        elif distance_cm < 10:
            HasTube = True
            ev3.screen.print("Has Tube")
            break
        else:
            HasTube = False
# -----------------------------
# MAIN PROGRAM FLOW
# ----------------------------
# FIRST INTERSECTION
robot.straight(100)
#grabber.run(speed=-800)
#wait(500)
#grabber.stop(Stop.HOLD)
elevator.run(600)
wait(40)
elevator.run_target(600, -130)
#stop_and_brake()
print("start following")
#follow_line_for_seconds(2)
while True:
    follow_line()
    distance_cm = ultrasonic_wall.distance() / 10
    if distance_cm < 20:
        print("Turn Right")
        break
ev3.speaker.beep()    # detect + stop
#wait_after_intersection(.3)
turn_right(80)
follow_line_for_seconds(1)
# SECOND INTERSECTION
follow_until_intersection()
wait_after_intersection(.5)
print("Turn Left")
turn_left(75)
print("Turned Left")
# FOLLOW LINE until other side
robot.straight(300)
follow_line_for_seconds(1)
print("Follow line for seconds")
follow_until_intersection()
print("Intersection met")
ev3.speaker.beep()    # detect + stop
for i in range(3):
    wait_after_intersection(2)
    follow_until_intersection()
    ev3.speaker.beep()    # detect + stop
# at inersection
follow_until_intersection()
wait_after_intersection(1)
turn_left(80)
follow_line_for_seconds(1)
follow_until_intersection()
# Release Grabber
elevator.run_target(300, 0, then = stop.COAST, wait = False)
grabber.run(speed=800)
HasTube = False
wait(2000)
grabber.stop(Stop.HOLD)

turn_right(140)
follow_line_for_seconds(2)
follow_until_intersection()
wait_after_intersection(.3)
turn_right(80)
ev3.speaker.beep()

# Fan
follow_line_for_seconds(1)
follow_until_intersection() 
ev3.speaker.beep()    # detect + stop
wait_after_intersection(.3)    
ev3.speaker.beep()  # move forward 3 seconds
turn_right(80)
follow_line_for_seconds(1)
follow_until_End()

line = 1
if HasTube == False: 
    turn_right(140)
    follow_until_intersection()
    wait_after_intersection(.3)    
    turn_right(80)
    follow_line_for_seconds(1)
    follow_until_intersection()
    wait_after_intersection(.3)    
    ev3.speaker.beep()  # move forward 3 seconds
    turn_right(80)
    follow_line_for_seconds(3)
    follow_until_End()
    line = 2
    if HasTube == False: 
        turn_right(140)
        follow_until_intersection()
        wait_after_intersection(.3)    
        turn_right(80)
        follow_line_for_seconds(2)
        follow_until_intersection()
        wait_after_intersection(.3)
        ev3.speaker.beep()  # move forward 3 seconds
        turn_right(80)
        follow_line_for_seconds(3)
        follow_until_End()
        line = 3
if HasTube == True:
        wait(500)
        stop_and_brake()
        grabber.run(speed=-800)
        wait(2000)
        grabber.stop(Stop.HOLD)
        elevator.run_angle(speed=100, rotation_angle=-720,then=Stop.HOLD, wait=False)
        wait(1000)
        turn_left(140)
        follow_line_for_seconds(2)
        follow_until_intersection()
        wait_after_intersection(.3)
        if line == 1:
            follow_line_for_seconds(6)
            grabber.run(speed=800)
            wait(2000)
            grabber.stop(Stop.HOLD)
        elif line == 2:
            follow_line_for_seconds(6)
            grabber.run(speed=800)
            wait(2000)
            grabber.stop(Stop.HOLD)
        elif line == 3:
            turn_left(80)
            follow_line_for_seconds(2)
            follow_until_intersection()
            wait_after_intersection(.3)
            turn_right(80)
            follow_line_for_seconds(6)
            grabber.run(speed=800)
            HasTube = False
            wait(2000)
            grabber.stop(Stop.HOLD)

robot.drive(-100,0)
wait(1200)
stop_and_brake()
elevator.run(speed=800)
wait(2000)
elevator.stop(Stop.HOLD)

turn_left(140)
robot.drive(-100,0)
wait(1000)
stop_and_brake()
wait(500)
follow_line_for_seconds(2)
follow_until_intersection()
wait_after_intersection(.3)    
turn_right(80)
follow_line_for_seconds(7)
follow_until_intersection()
wait_after_intersection(1) 
follow_until_End()

wait(500)
stop_and_brake()
grabber.run(speed=-800)
wait(2000)
grabber.stop(Stop.HOLD)

turn_right(140)
follow_line_for_seconds(.5)
follow_until_intersection()
wait_after_intersection(.3)
turn_right(80)
follow_line_for_seconds(1)
follow_until_intersection()

grabber.run(speed=800)
HasTube = False
wait(2000)
grabber.stop(Stop.HOLD)

turn_right(140)
follow_line_for_seconds(2)
follow_until_intersection()
wait_after_intersection(.3)
turn_right(80)
follow_line_for_seconds(5)
follow_until_intersection()
ev3.speaker.beep()    # detect + stop
for i in range(3):
    wait_after_intersection(2)
    follow_until_intersection()
    ev3.speaker.beep()    # detect + stop
    wait_after_intersection(2)
    follow_until_intersection()
    ev3.speaker.beep()    # detect + stop
    wait_after_intersection(2)
    follow_until_intersection()
    ev3.speaker.beep()    # detect + stop
wait_after_intersection(2)
follow_until_End()

wait(500)
stop_and_brake()
grabber.run(speed=-800)
wait(2000)
grabber.stop(Stop.HOLD)

turn_right(140)
follow_line_for_seconds(2)
follow_until_intersection()
wait_after_intersection(.3)
follow_line_for_seconds(5)
follow_until_intersection()
ev3.speaker.beep()    # detect + stop
for i in range(2):
    wait_after_intersection(2)
    follow_until_intersection()
    ev3.speaker.beep()    # detect + stop
    wait_after_intersection(2)
    follow_until_intersection()
    ev3.speaker.beep()    # detect + stop
wait_after_intersection(2)
# at inersection
follow_until_intersection()
wait_after_intersection(1)
turn_left(80)
follow_line_for_seconds(1)
follow_until_intersection()

grabber.run(speed=800)
HasTube = False
wait(2000)
grabber.stop(Stop.HOLD)

# Release Grabbe
turn_right(140)
follow_line_for_seconds(2)
follow_until_intersection()
wait_after_intersection(.3)
turn_right(80)