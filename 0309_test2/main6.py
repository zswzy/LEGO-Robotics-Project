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


def move(vx, vy, omega_deg, elapsed_time, motors, a=78, b=72, r=32):
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


def move_absolu(Vx, Vy, omega_deg, elapsed_time, theta_deg, motors):
    # move in earth reference
    # Vx, Vy: speed in earth reference
    # omega_deg: rotation speed in local reference: deg/s
    # elapsed_time: duration of the moving
    # theta_deg： angle between earth reference and local reference (positive angle if earth axe turn counterclockwise to local axe)
    # motors: a dictionnay of 4 motors
    # a,b,c: dimension

    # transition between earth reference and local reference
    theta   = theta_deg/180*math.pi
    vx      = Vx*math.cos(theta) + Vy*math.sin(theta)
    vy      = Vy*math.cos(theta) - Vx*math.sin(theta)

    move(vx,vy,omega_deg,elapsed_time,motors)

    return 0 

# Create your objects here.
ev3 = EV3Brick()
left_up_motor       = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_up_motor      = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_down_motor     = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
right_down_motor    = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
motors = {'left_up_motor':left_up_motor, 'right_up_motor':right_up_motor, 'left_down_motor':left_down_motor,'right_down_motor':right_down_motor}

Gyro=GyroSensor(Port.S1)

# Write your program here.
ev3.speaker.beep()

Gyro.reset_angle(0)

# mission1: reject disturbunce, keep walking in original direction
for i in range(20):
    theta=-Gyro.angle()
    print (theta)
    move_absolu(200,0,0,0.5,theta,left_up_motor, left_down_motor, right_up_motor, right_down_motor)


# mission2.1: walk a rectangle (no turn)
move(vx=500, vy=0, omega_deg=0, time_elpsed=2, motors)      # straight
move(vx=0, vy=500, omega_deg=0, time_elpsed=1.6,motors)     # left
move(vx=-500, vy=0, omega_deg=0, time_elpsed=2,motors)      # back
move(vx=0, vy=-500, omega_deg=0, time_elpsed=1.6,motors)    # right


# mission2.2: walk a rectangle (with turn)
for _ in range(4):
    move(vx=500, vy=0, omega_deg=0, time_elpsed=2, motors)  # stright
    move(vx=0, vy=0, omega_deg=45, time_elpsed=2, motors)   # turn left


