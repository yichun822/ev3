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


left_line_sensor = ColorSensor(Port.S1)
right_line_sensor = ColorSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

timenow=time.time()-1
# timenow=0
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
glonum=0

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
        # while left_sound_sensor.distance()<150 or right_sound_sensor.distance()<150:
        #     continue
        left_motor.run(0)
        right_motor.run(0)
        if normal=='left':
            targeting_motor.run_angle(500,400)
            targeting_motor.run_angle(500,-400)
        else:
            targeting_motor.run_angle(500,-400)
            targeting_motor.run_angle(500,400)

        
        
        
            
def xun(speed=100,kp=1.5,ki=0.001 ,kd=3,popnum=6,lstpop=[2,3,4,5]):
    global timenow
    DRIVE_SPEED = speed
    i=0
    last=0
    d=0

    # PROPORTIONAL_GAIN = gain

    # Start following the line endlessly.
    while True:
        
        # Calculate the deviation from the threshold.
        deviation = left_line_sensor.reflection() - right_line_sensor.reflection()
        
        # Calculate the turn rate.
        # turn_rate = PROPORTIONAL_GAIN * deviation
        i+=deviation
        d=deviation-last
        turn=(deviation*kp)+(i*ki)+(d*kd)
        left_motor.run(speed+turn)
        right_motor.run(speed-turn)
        # Set the drive base speed and turn rate.
        # robot.drive(DRIVE_SPEED, turn_rate)
        # timenow=time.time()
        n=sound(popnum,lstpop)
        
        # You can wait for a short time or do other things in this loop.
        wait(10)
        if left_line_sensor.reflection() < 15 and right_line_sensor.reflection() < 15 :
            left_motor.run(0)
            right_motor.run(0)
            for i in range(45):
                left_motor.run_angle(500,2)
                right_motor.run_angle(500,2)
            
            # left_motor.run_time(500, 70)
            # right_motor.run_time(500, 70)
        last = deviation 
        if n==1:
            break
lstpop=[2,3,4,5,6]
popnum=6
xun(300,4,0.001,3,popnum,lstpop)
