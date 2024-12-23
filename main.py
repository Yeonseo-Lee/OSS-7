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
gyro = GyroSensor(Port.S3)
ser = UARTDevice(Port.S2, baudrate=115200) # 공 tracking
toucn = TouchSensor(Port.S1) # touch sensor.

#==========[motors]==========
grab_motor = Motor(Port.C)
shooting_motor = Motor(Port.B)
gyro = GyroSensor(Port.S4)
ser = UARTDevice(Port.S3, baudrate=115200) # 공 trackingx``

#==========[motors]==========
grab_motor = Motor(Port.B)
shooting_motor = Motor(Port.C)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

#==========[target_angle turn(gyro)]==========
def turn(target_angle, power):
    
    # robot.drive(power, power)
    current_angle = gyro.angle()

    while abs(current_angle - target_angle) > 15:
        if current_angle < target_angle:
            robot.drive(0, power)
        else:
            robot.drive(0, -power)
        current_angle = gyro.angle()
        time.sleep(0.1)
    robot.stop()


#==========[grab and shooting]===========

def grab(command):
    if command == 'motion3':
        #close
        grab_motor.run(3350)
        time.sleep(0.20)
        grab_motor.stop()
        
        grab_motor.run(3350) 
        time.sleep(0.30)
        grab_motor.stop()
    elif command == 'motion1':
        grab_motor.run(-3350)
        time.sleep(0.30)
        grab_motor.stop()
        
    elif command == 'motion2':
        grab_motor.run(400)
        time.sleep(0.20)
        grab_motor.run(300)
        time.sleep(0.2)
        grab_motor.stop()

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run(-7750)
        time.sleep(0.25)
        shooting_motor.stop()

    elif command == 'shoot':
        #shooting
        shooting_motor.run(7850)
        time.sleep(0.2)
        shooting_motor.stop()

    elif command == 'shoot':
        #shooting
        shooting_motor.run(8000)
        time.sleep(0.25)
        shooting_motor.stop()


#==========[camera_chase]==========
def process_uart_data(data):
    try:
        # 데이터를 문자열로 디코드 (키워드 인자 제거)
        data_str = data.decode().strip()
        if not data_str:
            pass

        # 문자열에서 리스트 파싱
        data_str = data_str.strip("[]")
        parsed_list = [int(value.strip()) for value in data_str.split(",")]

        # 파싱된 결과 반환
        return parsed_list
    except:
        # 에러 처리
        return [-1,-1] # -1이 나오면 무시하는 코드 사용

def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output)
    previous_error = error

#==========[setup]==========
ev3.speaker.beep()
threshold = 80
previous_error = 0
gyro.reset_angle(0)

#==========[zero set position setting]==========
shoot('zero') #shoot 모터가 안쪽이고,
grab('motion3') #grab 모터가 바깥쪽이므로 shoot먼저 세팅 후 grab을 세팅해야한다
time.sleep(1)
grab('motion1') #공을 잡기 위한 높이로 열기

print("Zero set postion completed")

#==========[main loop]==========
while True:
    data = ser.read_all()
    # 데이터 처리 및 결과 필터링
    try:
        filter_result = process_uart_data(data) # sensor 값 (center x, center y)
        if filter_result[0] == -1 and filter_result[1] == -1:
            robot.drive(100, 0)   # 공이 안 보이면 보일 때까지 앞으로 직진
        elif filter_result[0]!= -1 and filter_result[1]!= -1:
            if filter_result[1] > 100: #공이 카메라 화면 기준으로 아래에 위치 = 로봇에 가까워졌다
                time.sleep(0.1)
                grab('motion3') #공을 잡기
                time.sleep(0.1) #동작간 딜레이
                turn(0, 50) #정면(상대방 진영)바라보기
                robot.straight(50)
                print('straight')
                time.sleep(0.1) #동작간 딜레이
        print(filter_result)
        if filter_result[0] == -1 and filter_result[1] == -1:
            robot.drive(100, 0)   # 공이 안 보이면 보일 때까지 앞으로 직진
        elif filter_result[0]!= -1 and filter_result[1]!= -1:
            if filter_result[1] > 85: #공이 카메라 화면 기준으로 아래에 위치 = 로봇에 가까워졌다
                time.sleep(0.2)
                grab('motion3') #공을 잡기
                time.sleep(0.2) #동작간 딜레이
                turn(0, 50) #정면(상대방 진영)바라보기
                robot.straight(50)
                print('straight')
                time.sleep(0.2) #동작간 딜레이
                print('motion1')
                robot.straight(-30)
                grab('motion1') #슛을 위한 열기
                time.sleep(0.1) #동작간 딜레이
                print('shoot')
                shoot('shoot') #공 날리기
                time.sleep(0.2) #동작간 딜레이
                shoot('shoot') #공 날리기
                time.sleep(0.1) #동작간 딜레이
                shoot('zero')
                print('motion2')
                grab('motion2')
            else: #공이 카메라 화면 기준 멀리 위치해 있으면 chase한다
                pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)

        time.sleep_ms(50)
    except:
        pass

