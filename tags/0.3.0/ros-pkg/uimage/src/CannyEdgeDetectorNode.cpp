/*
 * CannyEdgeDetectorNode.cpp
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <opencv2/imgproc/imgproc.hpp>

//#include <dynamic_reconfigure/server.h>
//#include <rtabmap_image/motionFilterConfig.h>

image_transport::Publisher rosPublisher;
//cv::Mat previousImage;
//double ratio = 0.2;

//void callback(rtabmap_image::motionFilterConfig &config, uint32_t level)
//{
//	ratio = config.ratio;
//}

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(rosPublisher.getNumSubscribers() && msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		cv::Mat dst;
		cv::Mat color_dst;
		cv::Canny( ptr->image, dst, 50, 200, 3 );
		cvtColor( dst, color_dst, CV_GRAY2BGR );


		cv_bridge::CvImage img;
		img.header.stamp = ptr->header.stamp;
		img.header.frame_id = ptr->header.frame_id;
		img.encoding = sensor_msgs::image_encodings::BGR8;
		img.image = color_dst;
		rosPublisher.publish(img.toImageMsg());

	}
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "canny_edge_detector");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);
	rosPublisher = it.advertise("image_edges", 1);

	//dynamic_reconfigure::Server<rtabmap_image::motionFilterConfig> server;
	//dynamic_reconfigure::Server<rtabmap_image::motionFilterConfig>::CallbackType f;
	//f = boost::bind(&callback, _1, _2);
	//server.setCallback(f);

	ros::spin();

	return 0;
}
