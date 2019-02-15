#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>


void imageSubscriber(const sensor_msgs::Image::ConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	ROS_INFO("Subscribed to the node: [%s]", msg->);
}

/*
void panSubscriber(const sensor_msgs::Image::ConstPtr& msg)
{
	//ROS_INFO("Subscribed to the node [%s]", msg->data.c_str);
}*/

int main(int argc, char **argv)
{
	ros::init(argc,argv,"bearing_estimator_");

	// Parameters
	ros::NodeHandle n;

	std::string camera_topic_name;
	// std::string pan_topic_name;
	n.param<std::string>("camera", camera_topic_name);
	// n.param<std::string>("pan", pan_topic_name); 

	// Subscribers
	ros::Subscriber image_sub = n.subscribe(camera_topic_name, 100, imageSubscriber);
	// ros::Subscriber pan_angle_sub = n.subscribe(pan_topic_name, 100, panSubscriber);

	ros::spin();

	return 0;

}
