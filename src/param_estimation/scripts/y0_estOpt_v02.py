#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosbag
#from geometry_msgs.msg import 
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np


bag = rosbag.Bag('./bag.bag')

counter = 0
gt_x_list = []
gt_y_list = []

raggio = 0.1575
y0 = 0.51
t_ratio = 0.025974
initial_x = -0.832142114639
initial_y = 0.426361680031
initial_theta = +1.1254603

Ts = 0.02;

odom_x_list = []
odom_y_list = []
odom_x_list_ref = []
odom_y_list_ref = []
pointLimit = 15000;

for msg in bag.read_messages(topics=['/gt_pose']):
   gt_x_list.append(msg[1].pose.position.x)
   gt_y_list.append(msg[1].pose.position.y)

M_PI = 3.14159265358979323846
x = initial_x
y = initial_y
theta = initial_theta
FirstExec = True
for msg in bag.read_messages(topics=['/sync_msgs']):
   	rpm_avg_l = - (msg[1].rpm_fl + msg[1].rpm_rl)/2
	rpm_avg_r = (msg[1].rpm_fr + msg[1].rpm_rr)/2
	Vl_m = (rpm_avg_l*2*M_PI*raggio)/60
	Vr_m = (rpm_avg_r*2*M_PI*raggio)/60
	Vl_r = Vl_m * t_ratio
	Vr_r = Vr_m * t_ratio
	Vx = (Vl_r + Vr_r)/2
	omega_z = (-Vl_r+Vr_r)/(2*y0)
	x = x + Vx*Ts*np.cos(theta+(omega_z*Ts)/2)
	y = y + Vx*Ts*np.sin(theta+(omega_z*Ts)/2)
	theta = theta + omega_z*Ts
	odom_x_list.append(x)
	odom_y_list.append(y)
	receivedTime = msg[1].header.stamp
	if FirstExec:
		previousTime = receivedTime
		Ts = 0.02
		FirstExec = False
	else:
		Ts = receivedTime - previousTime
		previousTime = receivedTime
	
for msg in bag.read_messages(topics=['/scout_odom']):
   odom_x_list_ref.append(msg[1].pose.pose.position.x)
   odom_y_list_ref.append(msg[1].pose.pose.position.y)



bag.close()

#for i in range(pointLimit):
#    if gt_x_list[i] == gt_x_list[i-1]:
#        gt_x_list[i-1] = "NULL"
#    print len(gt_x_list)


plt.plot(gt_x_list[:pointLimit],gt_y_list[:pointLimit])
plt.plot(odom_x_list[:pointLimit],odom_y_list[:pointLimit])
plt.plot(odom_x_list_ref[:pointLimit],odom_y_list_ref[:pointLimit])
plt.grid(color='gainsboro', linestyle='-')
plt.show()
    
    


