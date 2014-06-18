/*
 * RGB2MonoNode.cpp
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

image_transport::Publisher rosPublisher;

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(rosPublisher.getNumSubscribers() && msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		if(ptr->image.depth() == CV_8U && ptr->image.channels() == 3)
		{
			cv::Mat mono;
			cv::cvtColor(ptr->image, mono, cv::COLOR_BGR2GRAY);

			cv_bridge::CvImage img;
			img.header = ptr->header;
			img.encoding = sensor_msgs::image_encodings::MONO8;
			img.image = mono;
			rosPublisher.publish(img.toImageMsg());
		}
		else
		{
			ROS_WARN("Image format should be 8bits - 3 channels");
		}
	}
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "rgb2mono");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);
	rosPublisher = it.advertise("image_mono", 1);

	ros::spin();

	return 0;
}
