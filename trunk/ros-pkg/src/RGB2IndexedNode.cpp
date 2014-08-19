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
#include <utilite/UColorTable.h>
#include <dynamic_reconfigure/server.h>
#include <utilite/rgb2indConfig.h>

image_transport::Publisher rosPublisher;
UColorTable colorTable(UColorTable::kSize1024);

void callback(utilite::rgb2indConfig &config, uint32_t level)
{
	if(config.color_table_size == 14)
	{
		colorTable = UColorTable(UColorTable::kSize16777216);
	}
	else
	{
		colorTable = UColorTable(1<<(config.color_table_size+3));
	}
}

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(rosPublisher.getNumSubscribers() && msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		if(ptr->image.depth() == CV_8U && ptr->image.channels() == 3)
		{
			cv::Mat ind = ptr->image.clone();
			unsigned char * imageData = (unsigned char *)ind.data;
			int widthStep = ind.cols * ind.elemSize();
			for(int i=0; i<ind.rows; ++i)
			{
				for(int j=0; j<ind.cols; ++j)
				{
					unsigned char & b = imageData[i*widthStep+j*3+0];
					unsigned char & g = imageData[i*widthStep+j*3+1];
					unsigned char & r = imageData[i*widthStep+j*3+2];
					int index = (int)colorTable.getIndex(r, g, b);
					colorTable.getRgb(index, r, g , b);
				}
			}
			cv_bridge::CvImage img;
			img.header = ptr->header;
			img.encoding = ptr->encoding;
			img.image = ind;
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
	ros::init(argc, argv, "rgb2ind");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);
	rosPublisher = it.advertise("image_indexed", 1);

	dynamic_reconfigure::Server<utilite::rgb2indConfig> server;
	dynamic_reconfigure::Server<utilite::rgb2indConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
