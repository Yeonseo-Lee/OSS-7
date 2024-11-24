#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
left_motor = Motor(Port.C)
right_motor = Motor(Port.D)

wheel_diameter = 5.6
axle_track = 115

robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
sensor1 = GyroSensor(Port.S4)
grab = Motor(Port.A)
shoot = Motor(Port.B)

grab.run_until_stalled(-100, then = Stop.COAST, duty_limit = 50)
grab.reset_angle(0)
grab.run_target(-100, 100)
robot.straight(100)
grab.run_until_stalled(100, then = Stop.COAST, duty_limit = 50)
grab.reset_angle(0)
grab.run_until_stalled(-100, then = Stop.COAST, duty_limit = 50)

robot.straight(-50)
robot.turn(90)

shoot.run(2000)
time.sleep(0.25)
shoot.stop()
