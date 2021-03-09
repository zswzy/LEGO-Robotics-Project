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


def move(vx,vy,omega_deg,elapsed_time,motors,a=78, b=72, r=32):
    # move in robot local reference
    # vx, vy: speed, mm/s
    # omega_deg: rotation speed, positive rotation:counterclockwise, deg/s
    # elapsed_time: duration of the moving
    # motors: a dictionnay of 4 motors
    # a,b,c: dimension

    left_up_motor       = motors['left_up_motor']
    right_up_motor      = motors['right_up_motor']
    left_down_motor     = motors['left_down_motor']
    right_down_motor    = motors['right_down_motor']

    # calculate local angular speed
    omega   = omega_deg/180*math.pi
    angular_speed_left_up_motor     = 1/r*(vx-vy-(a+b)*omega)
    angular_speed_right_up_motor    = 1/r*(vx+vy+(a+b)*omega)
    angular_speed_left_down_motor   = 1/r*(vx+vy-(a+b)*omega)
    angular_speed_right_down_motor  = 1/r*(vx-vy+(a+b)*omega)
    
    # run motor
    left_up_motor.run(angular_speed_left_up_motor*180/math.pi)
    right_up_motor.run(angular_speed_right_up_motor*180/math.pi)
    left_down_motor.run(angular_speed_left_down_motor*180/math.pi)
    right_down_motor.run(angular_speed_right_down_motor*180/math.pi)

    time.sleep(elapsed_time)
    return 0 


def move_absolu(Vx,Vy,omega_deg,elapsed_time,theta_deg,left_up_motor, left_down_motor, right_up_motor, right_down_motor,a=78, b=72, r=32):
    # move in earth reference
    # Vx, Vy: speed in earth reference
    # omega_deg
    # elapsed_time: duration of the moving
    # motors: a dictionnay of 4 motors
    # a,b,c: dimension
    left_up_motor       = motors['left_up_motor']
    right_up_motor      = motors['right_up_motor']
    left_down_motor     = motors['left_down_motor']
    right_down_motor    = motors['right_down_motor']

    theta = theta_deg/180*math.pi
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

left_up_motor       = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_up_motor      = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_down_motor     = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
right_down_motor    = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
motors = {'left_up_motor':left_up_motor, 'right_up_motor':right_up_motor, 'left_down_motor':left_down_motor,'right_down_motor':right_down_motor}

Gyro=GyroSensor(Port.S1)

Gyro.reset_angle(0)

for i in range(20):
    theta=-Gyro.angle()
    print (theta)
    move_absolu(200,0,0,0.5,theta,left_up_motor, left_down_motor, right_up_motor, right_down_motor)



move(500,0,0,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(0,500,0,1.6,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(-500,0,0,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(0,-500,0,1.6,left_up_motor, left_down_motor, right_up_motor, right_down_motor)

move(500,0,0,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(0,0,45,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(500,0,0,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(0,0,45,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(500,0,0,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(0,0,45,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(500,0,0,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)
move(0,0,40,2,left_up_motor, left_down_motor, right_up_motor, right_down_motor)

