#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

#==========[Initialize]==========
#==========[sensors]==========
ev3 = EV3Brick()
gyro = GyroSensor(Port.S1)
ser = UARTDevice(Port.S2, baudrate=115200)

#==========[motors]==========
grab_motor = Motor(Port.A)
shooting_motor = Motor(Port.D)

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

    
   
def grab(command):
    if command == 'motion3':
        #close
        grab_motor.run(3350)
        time.sleep(0.30)
        grab_motor.stop()
    elif command == 'motion1':
        grab_motor.run(-3350)
        time.sleep(0.30)
        grab_motor.stop()
        
    elif command == 'motion2':
        grab_motor.run(500)
        time.sleep(0.3)
        grab_motor.stop()

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run(-7750)
        time.sleep(0.25)
        shooting_motor.stop()
    elif command == 'shoot':
        #shooting
        shooting_motor.run(7750)
        time.sleep(0.25)
        shooting_motor.stop()


ev3.speaker.beep()
threshold = 80
previous_error = 0
gyro.reset_angle(0)






        