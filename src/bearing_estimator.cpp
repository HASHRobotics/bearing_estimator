#include "ros/ros.h"
#include "std_msgs/String.h"


void imageSubscriber(const std_msgs::String::ConstPtr)
{
	ROS_INFO("Subscribed to the node: [%s]", msg->data.c_str);
}

void panSubscriber(const std_msgs::String::ConstPtr)
{
	ROS_INFO("Subscribed to the node [%s]", msg->data.c_str);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"bearing_estimator_");

	// Parameters
	ros::NodeHandle n_param;

	n_param.param<std::string>("camera", camera_topic_name);
	n_param.param<std::string>("pan", pan_topic_name); 

	// Subscribers
	ros::Subscriber image_sub = n_param.subscribe(camera_topic_name, 1000, imageSubscriber);
	ros::Subscriber pan_angle_sub = n_param.subscribe(pan_topic_name, 1000, panSubscriber);

	ros::spin();

	return 0;

}
