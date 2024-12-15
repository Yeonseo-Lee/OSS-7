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
gyro = GyroSensor(Port.S4)
ser = UARTDevice(Port.S3, baudrate=115200) # 공 tracking
color = ColorSensor(Port.S1) # 컬러 센서


#==========[motors]==========
grab_motor = Motor(Port.B)
shooting_motor = Motor(Port.C)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

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
        shooting_motor.run(7750)
        time.sleep(0.25)
        shooting_motor.stop()
def turn(target_angle, power):
    
    print('robot turn')
    robot.drive(power, power)
    current_angle = gyro.angle()
    print(current_angle)

    while abs(current_angle - target_angle) > 20:
        if current_angle < target_angle:
            robot.drive(0, power)
        else:
            robot.drive(0, -power)
        current_angle = gyro.angle()
        time.sleep(0.1)

    robot.stop()
def isGreen(k):
    r,g,b=k
    return g>5 and g>r and g>b

#==========[setup]==========
ev3.speaker.beep()
threshold = 80
previous_error = 0
gyro.reset_angle(0)
grab('motion1')
grab('motion2')


#==========[main loop]==========
while True:
    data = ser.read_all()
    # 데이터 처리 및 결과 필터링
    try:
        filter_result = process_uart_data(data) # sensor 값 (center x, center y)
        print(color.rgb(),filter_result)
        if color.rgb()==(1,4,0):
            ev3.speaker.beep()
            robot.drive(-50,0)
            time.sleep(0.9)
            turn(0,5)
        elif filter_result == [-1, -1]:
            robot.drive(200, 0)
            
            
            
        elif (filter_result[0]!= -1 and filter_result[1]!= -1):
            
            if filter_result[1] > 85 : #공이 카메라 화면 기준으로 아래에 위치 = 로봇에 가까워졌다
                robot.drive(100,0) #강제로 앞으로 이동
                grab('motion3') #공을 잡기
                time.sleep(1) #동작간 딜레이
                turn(0, 100) #정면(상대방 진영)바라보기
                time.sleep(1) #동작간 딜레이
                grab('motion1') #슛을 위한 열기
                time.sleep(1) #동작간 딜레이
                shoot('shoot') #공 날리기
                time.sleep(1) #동작간 딜레이
                shoot('zero')
                grab('motion2') 
                #time.sleep(3)
            else: #공이 카메라 화면 기준 멀리 위치해 있으면 chase한다
                pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)

        time.sleep_ms(50)
    except:
        pass
