#include "ros/ros.h"
#include "string.h"
#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <msg_filter/SpeedAndOdom.h>
#include <odometry/OdometryAndMethod.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


class VisualizerNode
{
	public:

	  	double rpm_avg_l;
	  	double rpm_avg_r;
	  	double deltaT;
	

		VisualizerNode()
		{

			joint_publisher = n.advertise<sensor_msgs::JointState>("joint_state", 1);
			path_publisher = n.advertise<nav_msgs::Path>("paths", 1);
			wheelspeed_subscriber = n.subscribe<msg_filter::SpeedAndOdom>("msg_filter", 1, &VisualizerNode::subCallback, this);
			estPathSub = n.subscribe<odometry::OdometryAndMethod>("est_odom", 1, &VisualizerNode::estPathCallback, this);
			scoutPathSub = n.subscribe<nav_msgs::Odometry>("scout_odom", 1, &VisualizerNode::scoutPathCallback, this);
			gtPathSub = n.subscribe<geometry_msgs::PoseStamped>("gt_pose", 1, &VisualizerNode::gtPathCallback, this);
		}

		void subCallback(const typename msg_filter::SpeedAndOdom::ConstPtr& receivedMsg);

		void estPathCallback(const typename odometry::OdometryAndMethod::ConstPtr& receivedMsg);
		void scoutPathCallback(const typename nav_msgs::Odometry::ConstPtr& receivedMsg);
		void gtPathCallback(const typename geometry_msgs::PoseStamped::ConstPtr& receivedMsg);

	protected:
		ros::NodeHandle n;
		ros::Subscriber wheelspeed_subscriber;
		ros::Subscriber estPathSub;
		ros::Subscriber scoutPathSub;
		ros::Subscriber gtPathSub;
		ros::Publisher joint_publisher;
		ros::Publisher path_publisher;

};
