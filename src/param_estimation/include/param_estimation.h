#include "ros/ros.h"
#include "string.h"
#include <param_estimation/Estimation.h>

template<typename PubType, typename SubType>
class EstimationNode
{
	public:

		double omega_z;
		double Vx;
		double rpm_avg_l;
		double rpm_avg_r;
		double Vl_m;
		double Vr_m;
		double raggio;
		
		param_estimation::Estimation est;

		EstimationNode() {}

		EstimationNode(std::string pubTopicName, std::string subTopicName, int queueSize)
		{
			publisher = n.advertise<PubType>(pubTopicName, queueSize);
			subscriber = n.subscribe<SubType>(subTopicName, queueSize, &EstimationNode::subCallback, this);

			n.getParam("/raggio", raggio);
		}

		void subCallback(const typename SubType::ConstPtr& receivedMsg);

	protected:

		ros::NodeHandle n;

		ros::Subscriber subscriber;
		ros::Publisher publisher;
		


};
