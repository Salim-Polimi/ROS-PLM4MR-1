#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosbag
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np

import tf

raggio = 0.1575

M_PI = 3.14159265358979323846

# BAG 1 
# bag = rosbag.Bag('./bag1+odometry.bag')
# y0 = np.arange(0.514, 0.517, 0.001)
# t_ratio = np.arange(37.2, 37.5, 0.1)
# pointLimit = 1550 # 1550 per fare fitting nella zona iniziale / 7000 totali
# gt_limit = 6000 # 6000 per fare fitting nella zona iniziale / 18000 totali

#BAG 2
bag = rosbag.Bag('./bag2+odometry.bag')
y0 = np.arange(0.40, 0.65, 0.05)
t_ratio = np.arange(37.2, 37.3, 0.1)
pointLimit = 1900 # 1900 per fare fitting nella zona iniziale / 3500 totali
gt_limit = 9000 # 9000 totali e per fare fitting nella zona iniziale 

# BAG 3
# bag = rosbag.Bag('./bag3+odometry.bag')
# y0 = np.arange(0.514, 0.515, 0.001)
# t_ratio = np.arange(37.2, 37.3, 0.1)
# pointLimit = 2750 # 2750 per fare fitting nella zona iniziale / 6000 totali
# gt_limit = 9000 # 6000 per fare fitting nella zona iniziale / 14000 totali 

gt_x_list = []
gt_y_list = []
odom_x_list_ref = []
odom_y_list_ref = []



#print("\n!!! whoever is running this WATCH LINES 36 AND 44/45!!!")

counterGT = 0
FirstExec = True
for msg in bag.read_messages(topics=['/gt_pose']):
	counterGT += 1
	if FirstExec:
		initial_x = msg[1].pose.position.x
		initial_y = msg[1].pose.position.y - 0.225 #offset notato sul grafico
		quaternion = (msg[1].pose.orientation.x, msg[1].pose.orientation.y, msg[1].pose.orientation.z, msg[1].pose.orientation.w)
		FirstExec = False
	gt_x_list.append(msg[1].pose.position.x)
	gt_y_list.append(msg[1].pose.position.y)
print("\nINITIAL VALUES: \n x= {} \n y= {}".format(initial_x, initial_y))
euler = tf.transformations.euler_from_quaternion(quaternion)
#print("ORIENTATION: \nroll= {} \npitch= {} \nyaw= {}".format(euler[0], euler[1], euler[2]))
initial_theta = euler[2]	# this is the right command
#initial_theta = -euler[2] 	# this is the workaround, without "-" the odometry starts opposite to gt_pose and i don't know why
print(" theta= {} \n".format(initial_theta))
#print("counterGT = {}".format(counterGT))

counterSC = 0
for msg in bag.read_messages(topics=['/sync_msgs']):
	counterSC += 1
	odom_x_list_ref.append(msg[1].odom.pose.pose.position.x)
	odom_y_list_ref.append(msg[1].odom.pose.pose.position.y)

#print("counterSC = {}".format(counterSC))


counter = 1
for j in range(len(t_ratio)):
	plt.figure()
	for i in range(len(y0)):
		print("{}: y0 = {}, t_ratio = {}".format(counter, y0[i], t_ratio[j]))
		counter += 1
		x = initial_x
		y = initial_y
		theta = initial_theta
		FirstExec = True
		odom_x_list = []
		odom_y_list = []
		counterEST = 0
		for msg in bag.read_messages(topics=['/sync_msgs']):
			counterEST += 1
		   	rpm_avg_l = - (msg[1].rpm_fl + msg[1].rpm_rl)/2
			rpm_avg_r = (msg[1].rpm_fr + msg[1].rpm_rr)/2
			Vl_m = (rpm_avg_l*2*M_PI*raggio)/60
			Vr_m = (rpm_avg_r*2*M_PI*raggio)/60
			Vl_r = Vl_m * (1/t_ratio[j])
			Vr_r = Vr_m * (1/t_ratio[j])
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

		#print("counterEST = {}".format(counterEST))
		
		plt.scatter(odom_x_list[:pointLimit],odom_y_list[:pointLimit], s=0.5, label='y0={}'.format(y0[i]))
		
	plt.scatter(gt_x_list[:gt_limit],gt_y_list[:gt_limit], s=0.5, label='gt_pose')
	#plt.scatter(odom_x_list_ref[:pointLimit],odom_y_list_ref[:pointLimit], s=0.5, label='scout')
	plt.legend()
	plt.title('t_ratio = {}'.format(t_ratio[j]))




plt.show()

bag.close()