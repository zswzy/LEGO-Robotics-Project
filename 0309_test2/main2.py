#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
import numpy

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

left_up_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_up_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_down_motor = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
right_down_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)


Gyro = GyroSensor(Port.S1,positive_direction=Direction.CLOCKWISE)

Gyro.reset_angle(0)

for i in range(3):
    Gyro_angle = Gyro.angle()
    #Gyro_speed = Gyro.speed()
    print('Gyro angle: ', Gyro_angle)
    #print('Gyro speed: ', Gyro_speed)
    left_up_motor.run(300)
    right_up_motor.run(300)
    left_down_motor.run(300)
    right_down_motor.run(300)
    time.sleep(1)

for i in range(5):
    Gyro_angle = Gyro.angle()
    #Gyro_speed = Gyro.speed()
    print('Gyro angle: ', Gyro_angle)
    #print('Gyro speed: ', Gyro_speed)
    left_up_motor.run(300)
    right_up_motor.run(-300)
    left_down_motor.run(300)
    right_down_motor.run(-300)
    time.sleep(1)



