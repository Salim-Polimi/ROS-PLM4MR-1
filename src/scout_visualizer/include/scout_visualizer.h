#include "ros/ros.h"
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <odometry/OdometryAndMethod.h>
#include <msg_filter/SpeedAndOdom.h>
#include <nav_msgs/Odometry.h>





class VisualizerNode
{
	protected:
		
		ros::NodeHandle n;

		ros::Subscriber subEstOdom;
		ros::Subscriber subScoutOdom;
		ros::Subscriber subGtPath;

		ros::Publisher pubEstOdom;
		ros::Publisher pubScoutOdom;
		ros::Publisher pubGtPath;

	public:
		
		

		VisualizerNode()
		{
			pubEstOdom = n.advertise<nav_msgs::Odometry>("est_odom_visual", 1);
			pubScoutOdom = n.advertise<nav_msgs::Odometry>("scoutOdom_visual", 1);
			pubGtPath = n.advertise<nav_msgs::Path>("gt_path", 1);

			subEstOdom = n.subscribe<odometry::OdometryAndMethod>("est_odometry", 1, &VisualizerNode::estOdomSubCallback, this);
			subScoutOdom = n.subscribe<msg_filter::SpeedAndOdom>("sync_msgs", 1, &VisualizerNode::scoutOdomSubCallback, this);
			subGtPath = n.subscribe<geometry_msgs::PoseStamped>("gt_pose", 1, &VisualizerNode::gtPathSubCallback, this);
		}

		

		void estOdomSubCallback(const odometry::OdometryAndMethod::ConstPtr& receivedMsg);
		void scoutOdomSubCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg);
		void gtPathSubCallback(const geometry_msgs::PoseStamped::ConstPtr& receivedMsg);
	


};