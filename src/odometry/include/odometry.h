#include "ros/ros.h"
#include "string.h"
#include <std_msgs/Time.h>
#include <geometry_msgs/TwistStamped.h>

#include "odometry/setResetOdom.h"

#include <odometry/paramConfig.h>
#include <dynamic_reconfigure/server.h>

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
		double raggio;
		double x;
		double y;
		double theta;
		// Speed estimation parameters
		double y0;
		double t_ratio;
		// Integration method enum
		int int_method;

		double Ts;
		bool FirstExec;

		OdometryNode() {}
		OdometryNode(std::string pubTopicName, std::string subTopicName, int queueSize)
		{
			twist_publisher = n.advertise<PubType>(pubTopicName, queueSize);
			odometry_publisher = n.advertise<nav_msgs::Odometry>("est_odometry", 1);
			subscriber = n.subscribe<SubType>(subTopicName, queueSize, &OdometryNode::subCallback, this);
			
			n.getParam("/raggio", raggio);
			n.getParam("/y0", y0);
			n.getParam("/t_ratio", t_ratio);
			
			// initial conditions for the integration
			if(n.getParam("/initial_pose/initial_x", x)){
				ROS_INFO("T'APPOST X STA A %f", x);
			}else{
				ROS_INFO("NON VA BRO X STA A %f", x);
			}
			n.getParam("/initial_pose/initial_y",y);
			n.getParam("/initial_pose/initial_theta", theta);

			// forse non dovremmo dichiarare questa roba qui, dovremmo dichiararla in protected e poi assegnare qui (set_service = n.adv...)
			set_service = n.advertiseService("setOdom", &OdometryNode::setOdom, this);
			reset_service = n.advertiseService("resetOdom", &OdometryNode::resetOdom, this);

			Ts=0;
			FirstExec=true;
		}

		void subCallback(const typename SubType::ConstPtr& receivedMsg);
		bool setOdom(odometry::setResetOdom::Request  &req, odometry::setResetOdom::Response &res);
		bool resetOdom(odometry::setResetOdom::Request  &req, odometry::setResetOdom::Response &res);
	protected:
		ros::NodeHandle n;
		ros::Subscriber subscriber;
		ros::Publisher twist_publisher;
		ros::Publisher odometry_publisher;

		ros::ServiceServer set_service;
		ros::ServiceServer reset_service;
};
