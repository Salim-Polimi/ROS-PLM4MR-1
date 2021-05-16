#include "ros/ros.h"
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


template<typename PubType, typename SubType, typename SubType2>
class FilterNode
{
	public:

		FilterNode() {}

		FilterNode( std::string pubTopicName, 
					int pubQueueSize,
					std::string subTopicName1, 
					std::string subTopicName2, 
					std::string subTopicName3, 
					std::string subTopicName4, 
					std::string subTopicName5,
					int subQueueSize
					)
		{
			sync_pub = n.advertise<PubType>(pubTopicName, pubQueueSize);

			motorFL_sub.subscribe(n, subTopicName1, subQueueSize);
		  	motorFR_sub.subscribe(n, subTopicName2, subQueueSize);
			motorRL_sub.subscribe(n, subTopicName3, subQueueSize);
			motorRR_sub.subscribe(n, subTopicName4, subQueueSize);
			scoutOdom_sub.subscribe(n, subTopicName5, subQueueSize);
			

			sync.reset(new Sync(MySyncPolicy(10), motorFL_sub, motorFR_sub, motorRL_sub, motorRR_sub, scoutOdom_sub));   
    		sync->registerCallback(boost::bind(&FilterNode::callback, this, _1, _2, _3, _4, _5));
		}

		void callback ( const typename SubType::ConstPtr& msg1_speed,
						const typename SubType::ConstPtr& msg2_speed,
              			const typename SubType::ConstPtr& msg3_speed,
              			const typename SubType::ConstPtr& msg4_speed,
              			const typename SubType2::ConstPtr& msg5_odom
              			);

	protected:
		
		ros::NodeHandle n;

		message_filters::Subscriber<SubType> motorFL_sub;
	  	message_filters::Subscriber<SubType> motorFR_sub;
		message_filters::Subscriber<SubType> motorRL_sub;
		message_filters::Subscriber<SubType> motorRR_sub;
		message_filters::Subscriber<SubType2> scoutOdom_sub;
		
		ros::Publisher sync_pub;

		typedef message_filters::sync_policies::ApproximateTime<SubType, SubType, SubType, SubType, SubType2> MySyncPolicy;
    	typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    	boost::shared_ptr<Sync> sync;
};