/*
 * CameraNode.cpp
 *
 *  Created on: 1 févr. 2010
 *      Author: labm2414
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

#include <utilite/UImageCapture.h>

#include <utilite/ULogger.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEventsManager.h>
#include <utilite/UDirectory.h>
#include <utilite/UFile.h>

#include <dynamic_reconfigure/server.h>
#include <utilite/CameraConfig.h>

class CameraWrapper : public UEventsHandler
{
public:
	// Usb device like a Webcam
	CameraWrapper(int usbDevice = 0,
			      float imageRate = 0,
				  unsigned int imageWidth = 0,
				  unsigned int imageHeight = 0) :
		camera_(0),
		seqNumber_(0),
		frameId_("camera")
	{
		ros::NodeHandle nh("~");
		nh.param("frame_id", frameId_, frameId_);
		image_transport::ImageTransport it(nh);
		rosPublisher_ = it.advertise("image", 1);
		startSrv_ = nh.advertiseService("start", &CameraWrapper::startSrv, this);
		stopSrv_ = nh.advertiseService("stop", &CameraWrapper::stopSrv, this);
		UEventsManager::addHandler(this);
	}

	virtual ~CameraWrapper()
	{
		if(camera_)
		{
			camera_->join(true);
			delete camera_;
		}
	}

	bool init()
	{
		seqNumber_ = 0;
		if(camera_)
		{
			return camera_->init();
		}
		return false;
	}

	void start()
	{
		if(camera_)
		{
			return camera_->start();
		}
	}

	bool startSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("Camera started...");
		if(camera_)
		{
			camera_->start();
		}
		return true;
	}

	bool stopSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("Camera stopped...");
		if(camera_)
		{
			camera_->kill();
		}
		return true;
	}

	void setParameters(int deviceId, double frameRate, int width, int height, const std::string & path, bool autoRestart, bool pause)
	{
		ROS_INFO("Parameters changed: deviceId=%d, path=%s frameRate=%f w/h=%d/%d autoRestart=%s pause=%s",
				deviceId, path.c_str(), frameRate, width, height, autoRestart?"true":"false", pause?"true":"false");
		if(camera_)
		{
			UVideoCapture * videoCam = dynamic_cast<UVideoCapture *>(camera_);
			UImageFolderCapture * imagesCam = dynamic_cast<UImageFolderCapture *>(camera_);

			if(imagesCam)
			{
				// images
				if(!path.empty() && UDirectory::getDir(path+"/").compare(UDirectory::getDir(imagesCam->getPath())) == 0)
				{
					imagesCam->setImageRate(frameRate);
					imagesCam->setImageSize(width, height);
					imagesCam->setAutoRestart(autoRestart);
					if(pause && !camera_->isPaused())
					{
						imagesCam->join(true);
					}
					else if(!pause && imagesCam->isPaused())
					{
						imagesCam->start();
					}
				}
				else
				{
					delete camera_;
					camera_ = 0;
				}
			}
			else if(videoCam)
			{
				if(!path.empty() && path.compare(videoCam->getFilePath()) == 0)
				{
					// video
					videoCam->setImageRate(frameRate);
					videoCam->setImageSize(width, height);
					videoCam->setAutoRestart(autoRestart);
					if(pause && !videoCam->isPaused())
					{
						videoCam->join(true);
					}
					else if(!pause && videoCam->isPaused())
					{
						videoCam->start();
					}
				}
				else if(path.empty() &&
						videoCam->getFilePath().empty() &&
						videoCam->getUsbDevice() == deviceId)
				{
					// usb device
					unsigned int w;
					unsigned int h;
					videoCam->getImageSize(w, h);
					if((int)w == width && (int)h == height)
					{
						videoCam->setImageRate(frameRate);
						videoCam->setAutoRestart(autoRestart);
						if(pause && !videoCam->isPaused())
						{
							videoCam->join(true);
						}
						else if(!pause && videoCam->isPaused())
						{
							videoCam->start();
						}
					}
					else
					{
						delete camera_;
						camera_ = 0;
					}
				}
				else
				{
					delete camera_;
					camera_ = 0;
				}
			}
			else
			{
				ROS_ERROR("Wrong camera type ?!?");
				delete camera_;
				camera_ = 0;
			}
		}

		if(!camera_)
		{
			if(!path.empty() && UDirectory::exists(path))
			{
				//images
				camera_ = new UImageFolderCapture(path, 1, false, frameRate, autoRestart, width, height);
			}
			else if(!path.empty() && UFile::exists(path))
			{
				//video
				camera_ = new UVideoCapture(path, frameRate, autoRestart, width, height);
			}
			else
			{
				if(!path.empty() && !UDirectory::exists(path) && !UFile::exists(path))
				{
					ROS_ERROR("Path \"%s\" does not exist (or you don't have the permissions to read)... falling back to usb device...", path.c_str());
				}
				//usb device
				camera_ = new UVideoCapture(deviceId, frameRate, autoRestart, width, height);
			}
			init();
			if(!pause)
			{
				start();
			}
		}
	}

protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("UImageEvent") == 0)
		{
			UImageEvent * e = (UImageEvent*)event;
			const cv::Mat & image = e->image();
			if(!image.empty() && image.depth() == CV_8U)
			{
				cv_bridge::CvImage img;
				if(image.channels() == 1)
				{
					img.encoding = sensor_msgs::image_encodings::MONO8;
				}
				else
				{
					img.encoding = sensor_msgs::image_encodings::BGR8;
				}
				img.image = image;
				sensor_msgs::ImagePtr rosMsg = img.toImageMsg();
				rosMsg->header.frame_id = frameId_;
				rosMsg->header.stamp = ros::Time::now();
				rosMsg->header.seq = ++seqNumber_;
				rosPublisher_.publish(rosMsg);
			}
		}
		else if(event->getClassName().compare("ULogEvent") == 0)
		{
			ULogEvent * e = (ULogEvent*)event;
			if(e->getCode() == ULogger::kWarning)
			{
				ROS_WARN("%s", e->getMsg().c_str());
			}
			else if(e->getCode() >= ULogger::kError)
			{
				ROS_ERROR("%s", e->getMsg().c_str());
			}
		}
	}

private:
	image_transport::Publisher rosPublisher_;
	UAbstractImageCapture * camera_;
	ros::ServiceServer startSrv_;
	ros::ServiceServer stopSrv_;
	int seqNumber_;
	std::string frameId_;
};

CameraWrapper * camera = 0;
void callback(utilite::CameraConfig &config, uint32_t level)
{
	if(camera)
	{
		camera->setParameters(config.device_id, config.frame_rate, config.width, config.height, config.video_or_images_path, config.auto_restart, config.pause);
	}
}

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
	ULogger::setEventLevel(ULogger::kWarning);

	ros::init(argc, argv, "camera");

	ros::NodeHandle nh("~");

	camera = new CameraWrapper(); // webcam device 0

	dynamic_reconfigure::Server<utilite::CameraConfig> server;
	dynamic_reconfigure::Server<utilite::CameraConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();

	//cleanup
	if(camera)
	{
		delete camera;
	}

	return 0;
}
