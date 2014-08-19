/*
Copyright (c) 2008-2014, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
		img.header = ptr->header;
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
