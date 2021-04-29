#include "ros/ros.h"
#include "string.h"
#include <std_msgs/Time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>

#include "odometry/setOdom.h"
#include "odometry/resetOdom.h"

#include <odometry/paramConfig.h>
#include <dynamic_reconfigure/server.h>

#include <odometry/OdometryAndMethod.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>

template<typename PubType, typename PubType2, typename SubType>
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
		geometry_msgs::Quaternion quaternion;
		// Speed estimation parameters
		double y0;
		double t_ratio;
		// Integration method string
		std_msgs::String method;

		double Ts;
		bool FirstExec;
		int int_method;

		OdometryNode() {}

		OdometryNode(std::string pubTopicName, std::string pubTopicName2, std::string subTopicName, int queueSize)
		{

			twist_publisher = n.advertise<PubType>(pubTopicName, queueSize);
			odometry_publisher = n.advertise<PubType2>(pubTopicName2, queueSize);
			subscriber = n.subscribe<SubType>(subTopicName, queueSize, &OdometryNode::subCallback, this);

			n.getParam("/raggio", raggio);
			n.getParam("/y0", y0);
			n.getParam("/t_ratio", t_ratio);

			
			// initial conditions for the integration
			if(n.getParam("/initial_pose/initial_x", x))
			{
				ROS_INFO("x=%f", x);
			}
			else
			{
				ROS_INFO("errore x=%f", x);
			}

			n.getParam("/initial_pose/initial_y",y);
			n.getParam("/initial_pose/initial_theta", theta);

			set_service = n.advertiseService("setOdom", &OdometryNode::setOdom, this);
			reset_service = n.advertiseService("resetOdom", &OdometryNode::resetOdom, this);

			method.data = "euler [DEFAULT]";


			f = boost::bind(&OdometryNode::responseCallback, this, _1, _2); //ATTENTO HAI CAMBIATO GLI ARGOMENTI DELLA CLASSE -MOLI
	        server.setCallback(f);

			Ts=0;
			FirstExec=true;
		}

		void subCallback(const typename SubType::ConstPtr& receivedMsg);
		bool resetOdom(odometry::resetOdom::Request  &req, odometry::resetOdom::Response &res);
		bool setOdom(odometry::setOdom::Request  &req, odometry::setOdom::Response &res);
		void responseCallback(odometry::paramConfig &config, uint32_t level);

	protected:
		ros::NodeHandle n;
		ros::Subscriber subscriber;
		ros::Publisher twist_publisher;
		ros::Publisher odometry_publisher;

		// services
		ros::ServiceServer set_service;
		ros::ServiceServer reset_service;

		//server
  		dynamic_reconfigure::Server<odometry::paramConfig> server;
		dynamic_reconfigure::Server<odometry::paramConfig>::CallbackType f;

		// tf
		tf::TransformBroadcaster br;
  		tf::Transform transform;
};
