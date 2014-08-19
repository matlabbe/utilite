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

#include <opencv2/highgui/highgui.hpp>

#include <utilite/UDirectory.h>
#include <utilite/UConversion.h>
#include <utilite/UMath.h>

#include <signal.h>

#include "ImageViewQt.hpp"

ImageViewQt * view = 0;
bool imagesSaved = false;
bool imagesCompressed = true;
int i = 0;

// assume bgr
QImage cvtCvMat2QImage(const cv::Mat & image, bool isBgr = true)
{
	QImage qtemp;
	if(!image.empty() && image.depth() == CV_8U && image.channels()==3)
	{
		const unsigned char * data = image.data;
		qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
		for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
		{
			for(int x = 0; x < image.cols; ++x)
			{
				QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
				if(isBgr)
				{
					*p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
				}
				else
				{
					*p = qRgb(data[x * image.channels()], data[x * image.channels()+1], data[x * image.channels()+2]);
				}
			}
		}
	}
	else if(image.depth() == CV_32F && image.channels()==1)
	{
		// Assume depth image (float in meters)
		const float * data = (const float *)image.data;
		float min=0, max=0;
		uMinMax(data, image.rows*image.cols, min, max);
		qtemp = QImage(image.cols, image.rows, QImage::Format_Indexed8);
		for(int y = 0; y < image.rows; ++y, data += image.cols)
		{
			for(int x = 0; x < image.cols; ++x)
			{
				uchar * p = qtemp.scanLine (y) + x;
				if(data[x] < min || data[x] > max || isnan(data[x]))
				{
					*p = 0;
				}
				else
				{
					*p = uchar(255.0f - ((data[x]-min)*255.0f)/(max-min));
				}
			}
		}
	}
	else if(!image.empty() && image.depth() != CV_8U)
	{
		printf("Wrong image format, 8UC3 or 32FC1\n");
	}
	return qtemp;
}

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		//ROS_INFO("Received an image size=(%d,%d)", ptr->image.cols, ptr->image.rows);

		if(imagesSaved)
		{
			std::string path = "./imagesSaved";
			if(!UDirectory::exists(path))
			{
				if(!UDirectory::makeDir(path))
				{
					ROS_ERROR("Cannot make dir %s", path.c_str());
				}
			}
			path.append("/");
			path.append(uNumber2Str(i++));
			if(imagesCompressed)
			{
				path.append(".png");
			}
			else
			{
				path.append(".bmp");
			}
			if(!cv::imwrite(path.c_str(), ptr->image))
			{
				ROS_ERROR("Cannot save image to %s", path.c_str());
			}
			else
			{
				ROS_INFO("Saved image %s", path.c_str());
			}
		}
		else if(view && view->isVisible())
		{
			if(ptr->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
			{
				// Process image in Qt thread...
				QMetaObject::invokeMethod(view, "setImage", Q_ARG(const QImage &, QImage(ptr->image.data, ptr->image.cols, ptr->image.rows, ptr->image.cols, QImage::Format_Indexed8).copy()));
			}
			else if(ptr->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0)
			{
				// Process image in Qt thread...
				QMetaObject::invokeMethod(view, "setImage", Q_ARG(const QImage &, cvtCvMat2QImage(ptr->image)));
			}
			else if(ptr->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)
			{
				// Process image in Qt thread...
				QMetaObject::invokeMethod(view, "setImage", Q_ARG(const QImage &, cvtCvMat2QImage(ptr->image, false)));
			}
			else if(ptr->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)
			{
				// Process image in Qt thread...
				QMetaObject::invokeMethod(view, "setImage", Q_ARG(const QImage &, cvtCvMat2QImage(ptr->image)));
			}
			else
			{
				ROS_WARN("Encoding \"%s\" is not supported yet (try \"bgr8\" or \"rgb8\")", ptr->encoding.c_str());
			}
		}
	}
}

void my_handler(int s){
	QApplication::closeAllWindows();
	QApplication::exit();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_view_qt", ros::init_options::AnonymousName);
	ros::NodeHandle pn("~");

	pn.param("images_saved", imagesSaved, imagesSaved);
	pn.param("images_compressed", imagesCompressed, imagesCompressed);
	ROS_INFO("images_saved=%s", imagesSaved?"true":"false");
	if(imagesSaved)
	{
		ROS_INFO("images_compressed=%d", imagesCompressed?1:0);
	}

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);

	if(!imagesSaved)
	{
		QApplication app(argc, argv);
		view = new ImageViewQt();
		view->setWindowTitle(ros::names::remap("image").c_str());
		view->show();

		// Catch ctrl-c to close the gui
		// (Place this after QApplication's constructor)
		struct sigaction sigIntHandler;
		sigIntHandler.sa_handler = my_handler;
		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);

		ROS_INFO("Waiting for images...");
		ros::AsyncSpinner spinner(1); // Use 1 thread
		spinner.start();
		app.exec();
		spinner.stop();

		delete view;
	}
	else
	{
		ros::spin();
	}


	return 0;
}
