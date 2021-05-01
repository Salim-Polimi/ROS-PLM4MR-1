#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosbag
#from geometry_msgs.msg import 
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np


bag = rosbag.Bag('./bag1new.bag')

counter = 0
gt_x_list = []
gt_y_list = []
odom_x_list = []
odom_y_list = []
pointLimit = 15000;

for msg in bag.read_messages(topics=['/gt_pose']):
   gt_x_list.append(msg[1].pose.position.x)
   gt_y_list.append(msg[1].pose.position.y)

for msg in bag.read_messages(topics=['/scout_odom']):
   odom_x_list.append(msg[1].pose.pose.position.x)
   odom_y_list.append(msg[1].pose.pose.position.y)

bag.close()

#for i in range(pointLimit):
#    if gt_x_list[i] == gt_x_list[i-1]:
#        gt_x_list[i-1] = "NULL"
#    print len(gt_x_list)


plt.plot(gt_x_list[:pointLimit],gt_y_list[:pointLimit])
plt.plot(odom_x_list[:pointLimit],odom_y_list[:pointLimit])
plt.grid(color='gainsboro', linestyle='-')
plt.show()
    
    


