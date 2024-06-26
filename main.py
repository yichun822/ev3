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
ev3 = EV3Brick()
left_sound_sensor=UltrasonicSensor(Port.S3)
right_sound_sensor=UltrasonicSensor(Port.S4)
targeting_motor=Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)


color1 = ColorSensor(Port.S1)
color2 = ColorSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

timenow=time.time()-1
# timenow=0
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
glonum=0
#用于识别标靶并射击，我知道这个逻辑写的很乱，但是真的没精力优化了
def sound(popnum,lstpop):
    global timepop
    global timenow
    global glonum
    
    normal='right' 
    timepop=time.time()
    if left_sound_sensor.distance()<100:
        normal='left'
    if left_sound_sensor.distance()<100  or right_sound_sensor.distance()<100 :
        if timepop-timenow<3:
            # print(timepop,timenow)
            return 0
        timenow=time.time()
        glonum+=1
        if glonum==popnum:
            print("done")
            if glonum in lstpop:
                left_motor.run(0)
                right_motor.run(0)
                ev3.speaker.beep()
                if normal=='left':
                    targeting_motor.run_angle(500,400)
                    targeting_motor.run_angle(500,-400)
                else:
                    targeting_motor.run_angle(500,-400)
                    targeting_motor.run_angle(500,400)
            return 1
        if glonum not in lstpop:
            return 0
        ev3.speaker.beep()
        left_motor.run(0)
        right_motor.run(0)
        if normal=='left':
            targeting_motor.run_angle(500,400)
            targeting_motor.run_angle(500,-400)
        else:
            targeting_motor.run_angle(500,-400)
            targeting_motor.run_angle(500,400)

def pid_line_follow(kp, ki, kd, max_speed,popnum,lstpop):
    #pid = PIDController(kp, ki, kd, setpoint)  # 创建PID控制器实例
    integral = 0
    derivative = 0
    prev_error = 0

    while True:
        s1 = color1.reflection()  # 假设color1是一个传感器对象，获取传感器反射值
        s2 = color2.reflection()  # 假设color2是另一个传感器对象，获取传感器反射值
        error = s1 - s2  # 计算反射值差异作为误差
        integral += error  # 计算积分项
        derivative = error - prev_error  # 计算微分项
        turn = kp * error + ki * integral + kd * derivative  # 计算总的控制输出
        prev_error = error  # 更新上一次误差
        #turn = pid.update(error)  # 使用PID控制器更新转向控制输出
        
        # 根据PID输出调整机器人速度和转向
        left_motor.run(max_speed + turn)  # 左轮速度
        right_motor.run(max_speed - turn)  # 右轮速度
        # timenow=time.time()
        n=sound(popnum,lstpop)
        
        # You can wait for a short time or do other things in this loop.
        #wait(10)
        if s1 < 15 and s2 < 15 :
            left_motor.run(0)
            right_motor.run(0)
            for i in range(45):
                left_motor.run_angle(500,2)
                right_motor.run_angle(500,2)
            
            # left_motor.run_time(500, 70)
            # right_motor.run_time(500, 70)
        #last = deviation 
        if n==1:
            break
lstpop=[2,3,4,5,6]
popnum=6
kp = 4  # 适当的比例系数
ki = 0.0001  # 适当的积分系数
kd = 8  # 适当的微分系数
setpoint = 30  # 适当的目标反射值
max_speed = 200  # 适当的最大速度
pid_line_follow(kp, ki, kd, max_speed,popnum,lstpop)

