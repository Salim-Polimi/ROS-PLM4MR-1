#include "ros/ros.h"
#include "string.h"

template<typename PubType, typename SubType>
class PubSubNode
{
	public:
		PubSubNode() {}
		PubSubNode(std::string pubTopicName, std::string subTopicName, int queueSize)
		{
			publisher = n.advertise<PubType>(pubTopicName, queueSize);
			subscriber = n.subscribe<SubType>(subTopicName, queueSize, &PubSubNode::subCallback, this);
		}
		void subCallback(const typename SubType::ConstPtr& receivedMsg);


	protected:
			ros::Subscriber subscriber;
			ros::Publisher publisher;
			ros::NodeHandle n;

};
