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
import socket
import os

class BigRobot:
   #---------------------- initialization-------------------------------------------------------
   def __init__(self,dimensions,motors,GPS_number,Gyro,Ultra,Yanse,PID_parameters,time_step):
      # define system
      self.ev3 = ev3 = EV3Brick()

      # define dimensions
      self.a = dimensions['a']
      self.b = dimensions['b']
      self.r = dimensions['r']

      # define motors
      self.left_up_motor       = motors['left_up_motor']
      self.right_up_motor      = motors['right_up_motor']
      self.left_down_motor     = motors['left_down_motor']
      self.right_down_motor    = motors['right_down_motor']

      # define GPS
      self.GPS_number = GPS_number

      # define Sensors
      self.Gyro = Gyro
      self.Gyro.reset_angle(0)
      self.Ultra = Ultra
      self.Yanse = Yanse

      # define PID parameters
      self.Kp_x = PID_parameters['x']['Kp']
      self.Ki_x = PID_parameters['x']['Ki']
      self.Kd_x = PID_parameters['x']['Kd']
      self.Kp_y = PID_parameters['y']['Kp']
      self.Ki_y = PID_parameters['y']['Ki']
      self.Kd_y = PID_parameters['y']['Kd']
      self.Kp_theta = PID_parameters['theta']['Kp']
      self.Ki_theta = PID_parameters['theta']['Ki']
      self.Kd_theta = PID_parameters['theta']['Kd']
      
      # define watch
      self.watch = StopWatch()

      # define time steo
      self.time_step = time_step

      # initialize position
      self.position_x = 0
      self.position_y = 0
   
   #---------------------move in local reference -------------------------------------------------
   def move(self,vx,vy,omega_deg):
      # move in robot local reference
      # vx, vy: speed, mm/s
      # omega_deg: rotation speed, positive rotation:counterclockwise, deg/s

      # calculate local angular speed
      omega_rad   = omega_deg/180*math.pi
      angular_speed_left_up_motor     = 1/r*(vx-vy-(self.a+self.b)*omega_rad)
      angular_speed_right_up_motor    = 1/r*(vx+vy+(self.a+self.b)*omega_rad)
      angular_speed_left_down_motor   = 1/r*(vx+vy-(self.a+self.b)*omega_rad)
      angular_speed_right_down_motor  = 1/r*(vx-vy+(self.a+self.b)*omega_rad)
      
      # run motor
      self.left_up_motor.run(angular_speed_left_up_motor*180/math.pi)
      self.right_up_motor.run(angular_speed_right_up_motor*180/math.pi)
      self.left_down_motor.run(angular_speed_left_down_motor*180/math.pi)
      self.right_down_motor.run(angular_speed_right_down_motor*180/math.pi)

      return self.watch.time()

   #-------------------move in global reference---------------------------------------------------
   def move_absolu(Vx,Vy,omega_deg,theta_deg):
      # move in earth reference
      # Vx, Vy: speed in earth reference
      # omega_deg:deg/s

      # reference transition
      theta_rad = theta_deg/180*math.pi
      vx = Vx*math.cos(theta_rad) + Vy*math.sin(theta_rad)
      vy = -Vx*math.sin(theta_rad) + Vy*math.cos(theta_rad)

      omega_rad = omega_deg/180*math.pi
      angular_speed_left_up_motor = 1/r*(vx-vy-(self.a+self.b)*omega_rad)
      angular_speed_right_up_motor = 1/r*(vx+vy+(self.a+self.b)*omega_rad)
      angular_speed_left_down_motor = 1/r*(vx+vy-(self.a+self.b)*omega_rad)
      angular_speed_right_down_motor = 1/r*(vx-vy+(self.a+self.b)*omega_rad)
      
      self.left_up_motor.run(angular_speed_left_up_motor*180/math.pi)
      self.right_up_motor.run(angular_speed_right_up_motor*180/math.pi)
      self.left_down_motor.run(angular_speed_left_down_motor*180/math.pi)
      self.right_down_motor.run(angular_speed_right_down_motor*180/math.pi)

      return self.watch.time()

   #-------------------------get ultrasonic distance----------------------------------------------
   def get_obstacle_distance(self):
      return self.Ultra.distance()
   #-------------------------get gyro angle----------------------------------------------
   def get_angle(self):
      return self.Gyro.angle

   #-------------------------get GPS position------------------------------------------------------
   def get_GPS(self):
      _socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      _socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
      PORT = 1730
      _socket.bind(('', PORT))
      Position_XYZ,address = _socket.recvfrom(2048)   #等待对方发送信息
      Position_XYZ = Position_XYZ.decode('utf-8')      #提取对方发送的信息
      _socket.close()

      #print(Position_XYZ_real)
      a = 1
      while a == 1:
         Position_XYZ_real=Position_XYZ
         if Position_XYZ_real[11] == str(self.GPS_number):
               self_position = Position_XYZ_real
               #print(self_position)
               self.position_x = float(self_position[16:23])
               for i in range(len(self_position)):
                  if self_position[i] == ',':
                     self.position_y = float(self_position[i+2:i+9])
                     break
               a = 0

      return self.position_x, self.position_y

   # ------------------------get pid output values-------------------------------------------------------
   def pid_output_x(self,error_present,error_last,error_integral):
      
      proportion = self.Kp_x * error_present
      integral = self.Ki_x * self.time_step*error_present + error_integral
      derivative = self.Kd_x * (error_present-error_last)/self.time_step
      output = proportion + integral + derivative

      return output
   
   def pid_output_y(self,error_present,error_last,error_integral):
      proportion = self.Kp_y * error_present
      integral = self.Ki_y * self.time_step*error_present + error_integral
      derivative = self.Kd_y * (error_present-error_last)/self.time_step
      output = proportion + integral + derivative
      
      return output
   
   def pid_output_theta(self,error_present,error_last,error_integral_last):
      proportion = self.Kp_theta * error_present
      integral = self.Ki_theta * self.time_step*error_present + error_integral
      derivative = self.Kd_theta * (error_present-error_last)/self.time_step
      output = proportion + integral + derivative
      
      return output

   #----------------------- define target distance ----------------------------------------------------------
   def get_distance(self,target_x,target_y):
      # get euclidien distance
      distance = math.sqrt((self.position_x - target_x)**2 + (self.position_y - target_y)**2)

      return distance
   # -----------------------define get error in x,y and theta
   def get_error(self,target_x,target_y,target_theta)
      error_x = target.x - self.position_x
      error_y = target.y - self.position_y
      error_theta = target.theta - self.position_theta

      return error_x,error_y,error_theta

   # ---------------------define missions---------------------------------------------------------------------
   # MOVE A SQUARE WITHOUT TURNING
   def move_square_no_turn(self, edge=1000, vx=100, vy=100):
      #vx,vy:mm/s
      #edge:mm

      time_per_edge = edge/vx*1000 # ms

      self.ev3.speaker.beep()
      self.ev3.speaker.say('start')
      # move forward
      self.watch.reset() # reset the watch
      t = self.watch.time()
      while t <= time_per_edge:
         t = self.move(vx,0,0)

      # move left
      self.watch.reset() # reset the watch
      t = self.watch.time()
      while t <= time_per_edge:
         t = self.move(0,vy,0)

      # move back
      self.watch.reset() # reset the watch
      t = self.watch.time()
      while t <= time_per_edge:
         t = self.move(-vx,0,0)

      # move right
      self.watch.reset() # reset the watch
      t = self.watch.time()
      while t <= time_per_edge:
         t = self.move(0,-vy,0)

      self.ev3.speaker.say('finish')

      return 0

   #MOVE A SQUARE WITH A 90 DEG TURN AT EACH CORNER
   def move_square_with_turn(self, edge=1000, vx = 100, vy = 100, omega_deg = 30):
      # edge:mm
      # vx, vy: mm/s, omega_deg: deg/s
      edge = 1000
      time_per_edge = edge/vx *1000 # ms
      time_per_turn = 90/omega_deg *1000 # ms

      self.ev3.speaker.beep()
      self.ev3.speaker.say('start')

      for _ in range(4):
         # move forward
         self.watch.reset() # reset the watch
         t = self.watch.time()
         while t <= time_per_edge:
            t = self.move(vx,0,0)
         # turn left
         self.watch.reset() # reset the watch
         t = self.watch.time()
         while t <= time_per_turn:
            t = self.move(0,0,omega_deg)

      self.ev3.speaker.say('finish')

      return 0

   #GO A STRAIGHT LINE IN GLOBAL REFERENCE
   def reject_disturbance(self, Vx=100, Vy=100, run_time=10*1000):

      self.ev3.speaker.beep()
      self.watch.reset() # reset the watch
      t = self.watch.time()
      while t <= run_time
         theta_deg = self.get_angle()
         t = self.move_absolu(Vx,Vy,0,theta_deg)

      return 0

   #KEEP DISTANCE TO A OBSTACLE IN X DIRECTION
   def keep_distance(self, target_distance=200, tracking_time=10*1000):
      # distance: mm
      # tracking_time: ms
      self.ev3.speaker.beep()
      self.watch.reset() # reset the watch
      start_time = self.watch.time()
      t = start_time
      while t <= tracking_time:
         obstacle_distance = self.get_obstacle_distance()
         error_present = obstacle_distance-target_distance
         error_last = error_present
         error_integral = 0
         vx = self.pid_output_x(self,error_present,error_last,error_integral)
         t = self.move(self,vx,0,0)

      return 0
   #KEEP AN ANGLE
   def trake_angle(self, target_angle=180, tracking_time=10*1000):
      # 
      self.ev3.speaker.beep()
      self.watch.reset() # reset the watch
      t = self.watch.time()
      theta_deg = self.get_angle()
      error_last = target_angle-theta_deg
      error_int = 0
      while t <= tracking_time:
         theta_deg = self.get_angle()
         error_present = target_angle-theta_deg
         error_int += error_present*self.time_step
         omega_deg = self.pid_output_theta(self,error_present,error_last,error_int)
         error_last = error_present

         t = self.move(0,0,omega_deg)

   #MOVE TO A TARGET (LOCAL)
   def move_to_target_local_simple(self,target,run_time=30*1000,limit_distance=50):
      # in local axes
      x_position_target = target['x']
      y_position_target = target['y']

      self.ev3.speaker.beep()
      self.watch.reset() # reset the watch
      start_time = self.watch.time()
      t = start_time

      # init
      x_position,y_position = self.get_GPS()
      error_last_x = x_position_target - x_position
      error_last_y = y_position_target - y_position
      error_int_x = 0
      error_int_y = 0

      while t <= run_time:
         x_position,y_position = self.get_GPS()
         # x control
         error_present_x = x_position_target - x_position
         error_int_x += error_present_x*self.time_step
         vx = self.pid_output_x(error_present_x,error_last_x,error_int_x)
         error_last_x = error_present_x
         # y control
         error_present_y = y_position_target - y_position
         error_int_y += error_present_y*self.time_step
         vy = self.pid_output_y(error_present_y,error_last_y,error_int_y)
         error_last_y = error_present_y

         t = self.move(vx,vy,0)

   #MOVE TO A TARGET(GLOBAL)
   def move_to_target_global_simple(self,target,run_time=30*1000,limit_distance=50):
      # in local axes
      x_position_target = target['x']
      y_position_target = target['y']

      self.ev3.speaker.beep()
      self.watch.reset() # reset the watch
      start_time = self.watch.time()
      t = start_time

      # init
      x_position,y_position = self.get_GPS()
      error_last_x = x_position_target - x_position
      error_last_y = y_position_target - y_position
      error_int_x = 0
      error_int_y = 0

      while t <= run_time:
         x_position,y_position = self.get_GPS()
         # x control
         error_present_x = x_position_target - x_position
         error_int_x += error_present_x*self.time_step
         Vx = self.pid_output_x(error_present_x,error_last_x,error_int_x)
         error_last_x = error_present_x
         # y control
         error_present_y = y_position_target - y_position
         error_int_y += error_present_y*self.time_step
         Vy = self.pid_output_y(error_present_y,error_last_y,error_int_y)
         error_last_y = error_present_y

         theta_deg = self.get_angle()
         t = self.move_absolu(Vx,Vy,0,theta_deg)



# ---------------------parametes---------------------------------------------------------------------------
dimensions = {'a':78,'b':72,'r':32}

left_up_motor       = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_up_motor      = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
left_down_motor     = Motor(Port.C, positive_direction=Direction.CLOCKWISE)
right_down_motor    = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
motors = {'left_up_motor':left_up_motor, 
         'right_up_motor':right_up_motor, 
         'left_down_motor':left_down_motor,
         'right_down_motor':right_down_motor}

GPS_number = 4
Ultra = UltrasonicSensor(Port.S2)
Gyro = GyroSensor(Port.S1)
Yanse = ColorSensor(Port.S3)

PID_parameters = {'x':{'Kp':0.6,'Ki':0,'Kd':0.1},
                  'y':{'Kp':0.6,'Ki':0,'Kd':0.1},
                  'theta':{'Kp':4,'Ki':0,'Kd':0}} 

time_step = 50

p22 = {'x':1400,'y':470}
p12 = {'x':3405,'y':450}
p00 = {'x':0,'y':0}
p13 = {'x':3400,'y':-1270}
p31 = {'x':-590,'y':2400}
p42 = {'x':-2580,'y':530}

# --------------------define robot------------------------------------------------------------------------
LEGO = BigRobot(dimensions,motors,GPS_number,Gyro,Ultra,Yanse,PID_parameters,time_step)

#LEGO.move_square_no_turn(edge=1000, vx=100, vy=100)

#LEGO.move_square_with_turn(edge=1000, vx=100, vy=100, omega_deg=30)

#LEGO.reject_disturbance(Vx=100, Vy=100, run_time=10*1000)

#LEGO.keep_distance(target_distance=200,tracking_time=10*1000)

#LEGO.trake_angle(target_angle=180, tracking_time=10*1000)

#LEGO.move_to_target_local_simple(p22, run_time=30*1000, limit_distance=50)

#LEGO.move_to_target_global_simple(p22, run_time=30*1000, limit_distance=50)