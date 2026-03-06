# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       cyril                                                        #
# 	Created:      2/3/2026, 10:25:16 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Configs
brain=Brain()

class Robot:
    def __init__(self):
        # constants
        self.drive_max_speed = 100
        self.drive_sensitivity = 3

        self.old_switch = 0
        self.stickstate = 0
        
        self.linear_up = 255
        self.linear_down = -255

        self.outtake_speed = 100
        self.outtake_time = 10

        self.intake_lift_speed = 100
        self.intake_lift_time = 10
        
        self.intake_speed = 100
        self.right_lift_speed = 100
        self.left_lift_speed = 100
        self.side_intake_speed = 100

        # ports

        left_drive_port = Ports.PORT8
        right_drive_port = Ports.PORT10
        outtake_port = Ports.PORT9
        intake_lift_port = brain.three_wire_port.g
        intake_port = brain.three_wire_port.b
        linear_port = brain.three_wire_port.f
        side_intake_port = brain.three_wire_port.e
        side_intake_port_b = brain.three_wire_port.d
        left_lifter_port = brain.three_wire_port.a
        right_lifter_port = brain.three_wire_port.h
        stick_port = brain.three_wire_port.c

        self.controller = Controller(PRIMARY)

        self.left_drive_smart = Motor(left_drive_port, GearSetting.RATIO_18_1, False)
        self.right_drive_smart = Motor(right_drive_port, GearSetting.RATIO_18_1, True)
        self.drivetrain = DriveTrain(self.left_drive_smart, self.right_drive_smart, 319.19, 295, 40, MM, 1)
        
        self.outtake = Motor(outtake_port, GearSetting.RATIO_18_1, False)
        self.intake = Motor29(intake_port, True)
        self.intake_lift = Motor29(intake_lift_port, False)
        self.right_lift = Motor29(right_lifter_port, False)
        self.left_lift = Motor29(left_lifter_port, True)
        self.side_intake = Motor29(side_intake_port, False)
        self.side_intake_b = Motor29(side_intake_port_b, True)
        self.stick = Pwm(stick_port)

        self.linear = Pwm(linear_port)
        

    def drive(self):
            forward_backward = self.controller.axis3.position()
            left_right = self.controller.axis4.position() / self.drive_sensitivity

            left_speed = forward_backward + left_right
            right_speed = forward_backward - left_right

            # Limit speeds to max speed
            left_speed = max(min(left_speed, self.drive_max_speed), -self.drive_max_speed)
            right_speed = max(min(right_speed, self.drive_max_speed), -self.drive_max_speed)

            self.left_drive_smart.set_velocity(left_speed, PERCENT)
            self.right_drive_smart.set_velocity(right_speed, PERCENT)

            self.left_drive_smart.spin(FORWARD)
            self.right_drive_smart.spin(FORWARD)

    def operate_outtake(self):
        if self.controller.buttonL1.pressing():
            self.outtake.set_velocity(self.outtake_speed, PERCENT)
            self.outtake.spin(FORWARD)
        elif self.controller.buttonL2.pressing():
            self.outtake.set_velocity(self.outtake_speed, PERCENT)
            self.outtake.spin(REVERSE)
        else:
            self.outtake.stop(BRAKE)
    def operate_intake_lift(self):
        if self.controller.buttonX.pressing():
            self.intake_lift.set_velocity(self.intake_lift_speed, PERCENT)
            self.intake_lift.spin(FORWARD)
        elif self.controller.buttonB.pressing():
            self.intake_lift.set_velocity(self.intake_lift_speed, PERCENT)
            self.intake_lift.spin(REVERSE)
        else:
            self.intake_lift.stop()

    def operate_intake(self):
        if self.controller.buttonR1.pressing():
            self.intake.set_velocity(self.intake_speed, PERCENT)
            self.intake.spin(FORWARD)
        elif self.controller.buttonR2.pressing():
            self.intake.set_velocity(self.intake_speed, PERCENT)
            self.intake.spin(REVERSE)
        else:
            self.intake.stop()
    
    def operate_side_intake(self):
        if self.controller.buttonY.pressing():
            self.side_intake.set_velocity(self.side_intake_speed, PERCENT)
            self.side_intake.spin(FORWARD)
            self.side_intake_b.set_velocity(self.side_intake_speed, PERCENT)
            self.side_intake_b.spin(FORWARD)
        else:
            self.side_intake.stop()
            self.side_intake_b.stop()

    def operate_linear(self):
        if self.controller.buttonUp.pressing():
            self.linear.state(self.linear_up, PERCENT)
        if self.controller.buttonDown.pressing():
            self.linear.state(self.linear_down, PERCENT)


    def operate_lifts(self):
        lift_speed = self.controller.axis2.position()
        if lift_speed > 20:
            self.stick.state(100)
            self.left_lift.set_velocity(self.left_lift_speed, PERCENT)
            self.left_lift.spin(FORWARD)
            self.right_lift.set_velocity(self.right_lift_speed, PERCENT)
            self.right_lift.spin(FORWARD)
        elif lift_speed < -20:
            self.stick.state(100)
            self.left_lift.set_velocity(self.left_lift_speed, PERCENT)
            self.left_lift.spin(REVERSE)
            self.right_lift.set_velocity(self.right_lift_speed, PERCENT)
            self.right_lift.spin(REVERSE)
        else:
            self.left_lift.stop()
            self.right_lift.stop()

    def flick(self):
        if self.controller.buttonA.pressing() != self.old_switch:
            if self.controller.buttonA.pressing():
                if self.stickstate == 0:
                    self.stick.state(-100)
                    self.stickstate = 1
                else:
                    self.stick.state(100)
                    self.stickstate = 0
            self.old_switch = self.controller.buttonA.pressing()
robot = Robot()
robot.linear.state(-225)
robot.stick.state(100)
while True:
    robot.drive()
    robot.operate_outtake()
    robot.operate_intake_lift()
    robot.operate_intake()
    robot.operate_side_intake()
    robot.operate_linear()
    robot.operate_lifts()
    robot.flick()