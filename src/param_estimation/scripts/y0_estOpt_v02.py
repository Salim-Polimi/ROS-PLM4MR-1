#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosbag
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np

import tf

bag = rosbag.Bag('./bag.bag')


raggio = 0.1575
t_ratio = 0.025974

M_PI = 3.14159265358979323846

y0 = np.arange(0.45, 0.55, 0.02)


gt_x_list = []
gt_y_list = []
odom_x_list_ref = []
odom_y_list_ref = []

pointLimit = 1000;

print("\n!!! whoever is running this WATCH LINES 44/45 !!!")

FirstExec = True
for msg in bag.read_messages(topics=['/gt_pose']):
	if FirstExec:
		initial_x = msg[1].pose.position.x
		initial_y = msg[1].pose.position.y
		quaternion = (msg[1].pose.orientation.x, msg[1].pose.orientation.y, msg[1].pose.orientation.z, msg[1].pose.orientation.w)
		FirstExec = False
	gt_x_list.append(msg[1].pose.position.x)
	gt_y_list.append(msg[1].pose.position.y)
print("\nINITIAL VALUES: \n x= {} \n y= {}".format(initial_x, initial_y))
euler = tf.transformations.euler_from_quaternion(quaternion)
#print("ORIENTATION: \nroll= {} \npitch= {} \nyaw= {}".format(euler[0], euler[1], euler[2]))
initial_theta = euler[2]	# this is the right command
#initial_theta = -euler[2] 	# this is the workaround, without "-" the odometry starts opposite to gt_pose and i don't know why
print(" theta= {}".format(initial_theta))

for msg in bag.read_messages(topics=['/scout_odom']):
    odom_x_list_ref.append(msg[1].pose.pose.position.x)
    odom_y_list_ref.append(msg[1].pose.pose.position.y)


for i in range(len(y0)):
	x = initial_x
	y = initial_y
	theta = initial_theta
	FirstExec = True
	odom_x_list = []
	odom_y_list = []
	for msg in bag.read_messages(topics=['/sync_msgs']):
	   	rpm_avg_l = - (msg[1].rpm_fl + msg[1].rpm_rl)/2
		rpm_avg_r = (msg[1].rpm_fr + msg[1].rpm_rr)/2
		Vl_m = (rpm_avg_l*2*M_PI*raggio)/60
		Vr_m = (rpm_avg_r*2*M_PI*raggio)/60
		Vl_r = Vl_m * t_ratio
		Vr_r = Vr_m * t_ratio
		Vx = (Vl_r + Vr_r)/2
		omega_z = (-Vl_r+Vr_r)/(2*y0[i])
		receivedTime = msg[1].header.stamp.to_sec()
		if FirstExec:
			previousTime = receivedTime
			Ts = 0.02
			FirstExec = False
		else:
			Ts = receivedTime - previousTime
			previousTime = receivedTime
		x = x + Vx*Ts*np.cos(theta+(omega_z*Ts)/2)
		y = y + Vx*Ts*np.sin(theta+(omega_z*Ts)/2)
		theta = theta + omega_z*Ts
		odom_x_list.append(x)
		odom_y_list.append(y)
	fig, (plot1, plot2) = plt.subplots(1, 2, sharex=True, sharey=True)
	fig.suptitle('y0 = {}'.format(y0[i]))
	plot1.set_title('est_odom(R) vs scout_odom(G)')
	plot1.plot(odom_x_list[:pointLimit],odom_y_list[:pointLimit], color='red')
	plot1.plot(odom_x_list_ref[:pointLimit],odom_y_list_ref[:pointLimit], color='green')
	plot1.grid(color='gainsboro', linestyle='-')
	plot2.set_title('est_odom(R) vs gt_pose(B)')
	plot2.plot(odom_x_list[:pointLimit],odom_y_list[:pointLimit], color='red')
	plot2.plot(gt_x_list[:pointLimit],gt_y_list[:pointLimit], color='blue')
	plot2.grid(color='gainsboro', linestyle='-')

plt.show()

bag.close()




#for i in range(pointLimit):
#    if gt_x_list[i] == gt_x_list[i-1]:
#        gt_x_list[i-1] = "NULL"
#    print len(gt_x_list)




