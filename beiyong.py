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

def pid_line_follow(kp, ki, kd, max_speed):
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
        # robot.drive(left_motor, right_motor)  # 控制机器人的驾驶

        if s1 < 10 and s2 < 10:  # 如果反射值低于某个阈值，说明机器人已经跟随线路或轨迹到达目标位置
            robot.stop()  # 停止机器人
            break

# 使用 pid_line_follow 函数
kp = 5 # 适当的比例系数
ki = 0.0001  # 适当的积分系数
kd = 8  # 适当的微分系数
max_speed = 200  # 适当的最大速度

pid_line_follow(kp, ki, kd, max_speed)  # 调用PID巡线控制函数