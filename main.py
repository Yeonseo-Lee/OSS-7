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


# Write your program here.
from pybricks.robotics import DriveBase

ev3 = EV3Brick()


left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
grab_motor= Motor(Port.B)

wheel_diameter = 5.6
axle_track = 115

robot = DriveBase(left_motor,right_motor,wheel_diameter,axle_track) 

grab_motor.run_until_stalled(-200,then = Stop.COAST,duty_limit = 50)


#  while True:
#      for i in range(4):
#          robot.turn(90, 100)
#          wait(100)
#     for i in range(2):
#         turn(180, -100)
#         wait(1000)
#     turn(360, 100)

# grab.run_until_stalled(-100, then = Stop.COAST, duty_limit = 50)
# grab.reset_angle(0)
# grab.run_until_stalled(100, then = Stop.COAST, duty_limit = 50)
# grab.reset_angle(0)
# grab.run_target(-100, -100)

# grab.run_until_stalled(-100, then = Stop.COAST, duty_limit = 50)
# shoot.run(2000)
# time.sleep(0.2)
# shoot.stop()
# shoot.run(-2000)
# time.sleep(0.2)
# shoot.stop()

shooting_motor.run_until_stalled(-100,stop.COAST,duty_limit = 50)
grab_motor.run_until_stalled(100,stop.COAST,duty_limit = 100)
grab_motor.reset_angle(0)

grab_motor.run_target(100,-100)

robot.straight(100)

grab_motor.run_until_stalled(200,stop.COAST,duty_limit = 50)

grab_motor.run_until_stalled(-200,stop.COAST,duty_limit = 50)
shooting_motor.run(2000)
time.sleep(0.25)
shooting_motor.stop()
