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
robot = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=151)
left_color = ColorSensor(Port.S4)
right_color = ColorSensor(Port.S3)
grabber = Motor(Port.C)     
elevator = Motor(Port.B)
pool_ultrasonic = UltrasonicSensor(Port.S2)
wall_ultrasonic = UltrasonicSensor(Port.S1)
fan_line = 1
# Write your program here.
ev3.speaker.beep()
robot.reset()

def line_following_for_ints(inter_amount, speed=200, distance = 0):
    interaections = 0
    acceleration = 1
    while inter_amount != interaections:
        if distance != 0 and robot.distance()/10/distance > .8:
            acceleration = 1 - (((robot.distance()/10) - (distance * .8))/(distance * .2) * .70)
            if acceleration < .30:
                acceleration = .30
        if left_color.color() == Color.BLACK and right_color.color() == Color.BLACK:
            ev3.speaker.beep() 
            interaections += 1
            wait(100)
        elif left_color.color() != Color.BLACK and right_color.color() != Color.BLACK:
            robot.drive(speed * acceleration, 0)
        elif left_color.color() == Color.BLACK:
            robot.drive(speed * acceleration, -44)
        elif right_color.color() == Color.BLACK:
            robot.drive(speed * acceleration, 44)
        elif left_color.color() != Color.BLACK and right_color.color() != Color.BLACK and left_color.color() != Color.WHITE and right_color.color() != Color.WHITE:
             robot.straight(-15)

    robot.stop()
    robot.reset()
def follow_line_pool(speed= 150, distance= 0):
    while pool_ultrasonic.distance()/10 > 7:
        acceleration = 1
        if distance != 0 and robot.distance()/10/distance > .8:
            acceleration = 1 - (((robot.distance()/10) - (distance * .8))/(distance * .2) * .70)
            if acceleration < .30:
                acceleration = .30
        if left_color.color() == Color.BLACK and right_color.color() == Color.BLACK:
            robot.drive(speed * acceleration, 0)   
        elif left_color.color() != Color.BLACK and right_color.color() != Color.BLACK:
            robot.drive(speed * acceleration, 0)
        elif left_color.color() == Color.BLACK:
            robot.drive(speed * acceleration, -44)
        elif right_color.color() == Color.BLACK:
            robot.drive(speed * acceleration, 44)
        elif left_color.color() != Color.BLACK and right_color.color() != Color.BLACK and left_color.color() != Color.WHITE and right_color.color() != Color.WHITE:
             break
    robot.stop()
def follow_line_for_time(time, speed=200):
    watch = StopWatch()
    while watch.time() < time:
        if left_color.color() == Color.BLACK and right_color.color() == Color.BLACK:
            robot.drive(speed, 0)   
        elif left_color.color() != Color.BLACK and right_color.color() != Color.BLACK:
            robot.drive(speed, 0)
        elif left_color.color() == Color.BLACK:
            robot.drive(speed, -44)
        elif right_color.color() == Color.BLACK:
            robot.drive(speed, 44)
    robot.stop()

def grabber_close():
    grabber.run(speed=-800)
    wait(800)
    grabber.stop(Stop.HOLD)

def grabber_open():
    grabber.run(speed=800)
    wait(800)
    grabber.stop(Stop.HOLD)

def elevator_raise():
    elevator.run(600)
    wait(40)
    elevator.run_target(600, -130)

def elevator_lower():
    elevator.run_time(-300, 1000, then=Stop.COAST, wait=True)  

#region
follow_line_until_pool(100, 30)
grabber_close()
robot.turn(90)
robot.straight(130)
if pool_ultrasonic.distance()/10 < 10:
    ev3.speaker.beep()
    robot.straight(10)
robot.turn(90)
robot.straight(60)
robot.turn(90)
robot.straight(30)
if left_color.color() != Color.BLACK and right_color.color() != Color.BLACK:
    robot.drive(40, 0)
elevator_raise()
line_following_for_ints(1, 200, 43)
robot.straight(10)
robot.turn(-90)
robot.straight(10)
#endregion
#region
line_following_for_ints(4, 200, 132)
robot.straight(10)
robot.turn(-90)
robot.straight(10)
elevator_lower()
wait(500)
line_following_for_ints(1, 150, 20.5)
wait(500)
robot.straight(100)
grabber_open()
wait(500)
robot.straight(-100)
elevator_raise()
robot.turn(180)
line_following_for_ints(1, 150, 20)
#endregion

#region
robot.turn(-90)
robot.straight(-120)
elevator_lower()
wait(500)
follow_line_pool()
grabber_close()
wait(500)
elevator_raise()
robot.turn(180)
line_following_for_ints(1, 150, 10)
robot.straight(15)
robot.turn(90)
robot.straight(15)
elevator_lower()
wait(500)
line_following_for_ints(1, 150, 20.5)
robot.straight(80)
grabber_open()
wait(500)
robot.straight(-80)
elevator_raise()
robot.turn(180)
line_following_for_ints(1, 150, 20)
robot.straight(10)
robot.turn(90)



robot.straight(30)
for i in range(3):
    if i == 0:
        line_following_for_ints(1, 200, 48)
    else:
        line_following_for_ints(1, 200, 18)
    robot.straight(-40)
    wait(1000)
    if wall_ultrasonic.distance()/10 < 20:
        fan_line = i+1
        ev3.speaker.beep()
        wait(500)
        ev3.speaker.beep()
        robot.straight(70)
        break
    robot.straight(70)
robot.straight(30)
elevator_lower()
wait(500)

follow_line_pool()
grabber_close()
wait(500)
elevator_raise()
robot.turn(-180)
line_following_for_ints(4, 200, 160)
robot.turn(-90)
line_following_for_ints(1, 200, 20.5)
elevator_lower()
wait(500)
robot.straight(60)
grabber_open()
robot.straight(-60)
robot.turn(180)
line_following_for_ints(1, 200, 20)
robot.turn(90)
robot.straight(30)
if fan_line == 1:
    line_following_for_ints(1, 200, 47)
if fan_line == 2:
    line_following_for_ints(2, 200, 67)
if fan_line == 3:
    line_following_for_ints(3, 200, 87)
robot.turn(90)
follow_line_pool()
robot.straight(30)
grabber_close()
wait(500)
elevator_raise()
robot.straight(-50)
robot.turn(180)
line_following_for_ints(1, 200, 20)
if fan_line == 3:
    robot.turn(-90)
    line_following_for_ints(1, 200, 20)
    robot.turn(90)
robot.straight(30)
follow_line_for_time(1000)
grabber_open()
wait(500)
robot.straight(-50)
robot.turn(180)
line_following_for_ints(1)
robot.turn(90)
robot.straight(30)
line_following_for_ints(1)
if fan_line == 1:
    line_following_for_ints(1)

while wall_ultrasonic.distance()/10 < 36:
    follow_line_for_time(1000)
follow_line_for_time(2000)
robot.turn(-90)
robot.straight(30)
elevator_lower()
line_following_for_ints(1)
time = 0
while time != 3:
    distance_cm = ultrasonic.distance() / 10
    if distance_cm < 6.6:
        time += 1
        ev3.speaker.beep()
        wait(1000)
grabber_close()
robot.straight(-30)
robot.turn(180)
robot_follow_line_for_time(2000)
robot.turn(90)
robot.straight(30)
line_following_for_ints(1)
robot.straight(30)
robot.turn(-90)
line_following_for_ints(4)
robot.turn(-90)
robot.straight(30)
