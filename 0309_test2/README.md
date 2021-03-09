## main6.py
函数：
- move(vx, vy, omega_deg, elapsed_time, motors, a=78, b=72, r=32):
	- move in robot local reference
	- vx, vy: speed, mm/s
	- omega_deg: rotation speed, positive rotation:counterclockwise, deg/s
	- elapsed_time: duration of the moving
	- motors: a dictionnay of 4 motors
	- a,b,c: dimension, in mm
- move_absolu(Vx, Vy, omega_deg, elapsed_time, theta_deg, motors)
	- move in earth reference
	- Vx, Vy: speed in earth reference
	- omega_deg: rotation speed in local reference: deg/s
	- elapsed_time: duration of the moving
	- theta_deg： angle between earth reference and local reference (positive angle if earth axe turn counterclockwise to local axe)
	- motors: a dictionnay of 4 motors

主程序：
- 'beep'
- mission1: reject disturbunce, keep walking in original direction
- mission2.1: walk a rectangle (no turn)
- mission2.2: walk a rectangle (with turn)


