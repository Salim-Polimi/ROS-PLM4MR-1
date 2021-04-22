#include "ros/ros.h"
#include "string.h"
#include <std_msgs/Time.h>
#include <geometry_msgs/TwistStamped.h>

template<typename PubType, typename SubType>
class OdometryNode
{
	public:
		// Motor to velocities computation variables
		double omega_z;
	  double Vx;
	  double rpm_avg_l;
	  double rpm_avg_r;
		double Vl_m;
		double Vr_m;
		double Vl_r;
		double Vr_r;
		geometry_msgs::TwistStamped twist_msg;
		// integration variables
		double receivedTime;
		double previousTime;
		double x;
		double y;
		double theta;
		double Ts;
		bool FirstExec;

		OdometryNode() {}
		OdometryNode(std::string pubTopicName, std::string subTopicName, int queueSize)
		{
			twist_publisher = n.advertise<PubType>(pubTopicName, queueSize);
			odometry_publisher = n.advertise<nav_msgs::Odometry>("est_odometry", 1);
			subscriber = n.subscribe<SubType>(subTopicName, queueSize, &OdometryNode::subCallback, this);
			// initial conditions for the integration
			x=0;
			y=0;
			theta=0;
			Ts=0;
			FirstExec=true;
		}

		void subCallback(const typename SubType::ConstPtr& receivedMsg);
	protected:
			ros::Subscriber subscriber;
			ros::Publisher twist_publisher;
			ros::Publisher odometry_publisher;
			ros::NodeHandle n;
};
