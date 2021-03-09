#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
import math

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

a = 78
b = 72
r = 32
def move(vx,vy,omega_deg,elapsed_time,left_up_motor, left_down_motor, right_up_motor, right_down_motor,a=78, b=72, r=32):
    
    omega = omega_deg/180*math.pi
    angular_speed_left_up_motor = 1/r*(vx-vy-(a+b)*omega)
    angular_speed_right_up_motor = 1/r*(vx+vy+(a+b)*omega)
    angular_speed_left_down_motor = 1/r*(vx+vy-(a+b)*omega)
    angular_speed_right_down_motor = 1/r*(vx-vy+(a+b)*omega)
    
    left_up_motor.run(angular_speed_left_up_motor*180/math.pi)
    right_up_motor.run(angular_speed_right_up_motor*180/math.pi)
    left_down_motor.run(angular_speed_left_down_motor*180/math.pi)
    right_down_motor.run(angular_speed_right_down_motor*180/math.pi)
    time.sleep(elapsed_time)

    return 0 


def move_absolu(Vx,Vy,omega_deg,elapsed_time,left_up_motor, left_down_motor, right_up_motor, right_down_motor,Gyro,a=78, b=72, r=32):
    Gyro.reset_angle(0)
    theta=Gyro.angle()
    vx = Vx*math.cos(theta) + Vy*math.sin(theta)
    vy = Vy*math.cos(theta) - Vx*math.sin(theta)

    omega = omega_deg/180*math.pi
    angular_speed_left_up_motor = 1/r*(vx-vy-(a+b)*omega)
    angular_speed_right_up_motor = 1/r*(vx+vy+(a+b)*omega)
    angular_speed_left_down_motor = 1/r*(vx+vy-(a+b)*omega)
    angular_speed_right_down_motor = 1/r*(vx-vy+(a+b)*omega)
    
    left_up_motor.run(angular_speed_left_up_motor*180/math.pi)
    right_up_motor.run(angular_speed_right_up_motor*180/math.pi)
    left_down_motor.run(angular_speed_left_down_motor*180/math.pi)
    right_down_motor.run(angular_speed_right_down_motor*180/math.pi)
    time.sleep(elapsed_time)

    return 0 

# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

left_up_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_up_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_down_motor = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
right_down_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)

Gyro=GyroSensor(Port.S1)

#Gyro.reset_angle(0)

move_absolu(500,0,0,10,left_up_motor, left_down_motor, right_up_motor, right_down_motor,Gyro)



# move(500,0,0,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
# move(0,500,0,1.6,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
# move(-500,0,0,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
# move(0,-500,0,1.6,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)

#move(500,0,0,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
#move(0,0,45/180*math.pi,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
#move(500,0,0,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
#move(0,0,45/180*math.pi,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
#move(500,0,0,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
#move(0,0,45/180*math.pi,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
#move(500,0,0,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
#move(0,0,40/180*math.pi,2,a,b,r,left_up_motor, left_down_motor, right_up_motor, right_down_motor)

