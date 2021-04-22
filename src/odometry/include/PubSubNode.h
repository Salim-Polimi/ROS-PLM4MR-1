#include "ros/ros.h"
#include "string.h"
#include <std_msgs/Time.h>

template<typename PubType, typename SubType>
class PubSubNode
{
	public:
		double x;
		double y;
		double theta;
		double Ts;
		bool FirstExec;

		PubSubNode() {}
		PubSubNode(std::string pubTopicName, std::string subTopicName, int queueSize)
		{
			publisher = n.advertise<PubType>(pubTopicName, queueSize);
			euler_publisher = n.advertise<nav_msgs::Odometry>("est_odometry", 1);
			subscriber = n.subscribe<SubType>(subTopicName, queueSize, &PubSubNode::subCallback, this);
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
			ros::Publisher publisher;
			ros::Publisher euler_publisher;
			ros::NodeHandle n;
};
