/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "utilite/ULogger.h"
#include "utilite/UTimer.h"
#include "utilite/UDirectory.h"
#include "utilite/UFile.h"
#include "utilite/UConversion.h"
#include "utilite/UImageCapture.h"
#include "utilite/UEventsManager.h"
#include "utilite/UEventsHandler.h"

#include "utilite/UImageView.h"
#include "utilite/UCv2Qt.h"

#include <QtGui/QApplication>
#include <QtCore/QMetaObject>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef WIN32
	#include <signal.h>
#endif
#include <iostream>
#include <fstream>

void showUsage()
{
	printf("Usage:\n"
			"webcamCapture [option]\n"
			"   -device #:			Id of the webcam, a path to an image directory "
			"                         or a video file (default 0)\n"
			"   -width #:			Image width (default 640)\n"
			"   -height #:			Image height (default 480)\n"
			"   -fps #.#:			Frame rate (Hz) (default 30.0)\n"
			"   -hide :			    Image not shown while capturing (default false)\n"
			"   -saveto \"path\":	Name of the directory/video to save images\n"
			"   -saveext \"ext\":	Image extension (default \"jpg\", use \"avi\" for video)\n"
			"   -debug:				Debug trace\n");
	exit(1);
}

class ImageHandler : public UEventsHandler
{
public:
	ImageHandler(const std::string & targetDir, const std::string & ext, bool show, double fps) :
		targetDir_(targetDir),
		ext_(ext),
		save_(targetDir.size()),
		id_(1),
		view_(0),
		writer_(0),
		fps_(fps>0.0?fps:30.0)
	{
		if(show)
		{
			view_ = new UImageViewWidget();
			view_->show();
		}
	}
	virtual ~ImageHandler()
	{
		if(view_)
		{
			delete view_;
		}
		if(writer_)
		{
			writer_->release();
			delete writer_;
		}
	}

protected:
	virtual void handleEvent(UEvent * e)
	{
		if(e->getClassName().compare("UImageEvent") == 0)
		{
			const UImageEvent * event = (const UImageEvent*)e;
			cv::Mat image = event->image();
			if(!image.empty())
			{
				if(view_)
				{
					// Process image in Qt thread...
					QMetaObject::invokeMethod(view_, "setImage", Q_ARG(const QImage &, cvMat2QImage(image)));
				}

				if(save_ && id_ == 1)
				{
					//init
					if(save_ && ext_.compare("avi")==0)
					{
						targetDir_.append(std::string(".").append(ext_));
						printf("Saving to video file %s\n", targetDir_.c_str());
						writer_ = new cv::VideoWriter(targetDir_, CV_FOURCC('M','J','P','G'), fps_, cv::Size(image.cols, image.rows));
						if(!writer_->isOpened())
						{
							printf("Cannot create video file %s\n", targetDir_.c_str());
							delete writer_;
							writer_= 0;
							save_ = false;
						}
					}
					else if(save_)
					{
						UDirectory::makeDir(targetDir_);
					}
				}

				if(save_)
				{
					if(writer_)
					{
						writer_->write(image);
						printf("Image %d saved!\n", id_++);
					}
					else
					{
						std::string fileName = targetDir_ + "/";
						fileName += uNumber2Str(id_++);
						fileName += ".";
						fileName += ext_;
						cv::imwrite(fileName, image);
						printf("Image %s saved!\n", fileName.c_str());
					}
				}
			}
			else
			{
				printf("Image is null, stream terminated?!?\n");
			}
		}

	}

private:


private:
	std::string targetDir_;
	std::string ext_;
	bool save_;
	int id_;
	UImageViewWidget * view_;
	cv::VideoWriter * writer_;
	double fps_;
};

#ifndef WIN32
void my_handler(int s){
	QApplication::closeAllWindows();
	QApplication::exit();
}
#endif

int main(int argc, char * argv[])
{

	QApplication app(argc, argv);

	bool show = true;
	int usbDevice = 0;
	std::string path;
	int imageWidth = 640;
	int imageHeight = 480;
	float imageRate = 30.0;
	std::string extension = "jpg";
	std::string targetDirectory = "";

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-h") == 0 ||
			strcmp(argv[i], "-help") == 0 ||
			strcmp(argv[i], "?") == 0)
		{
			showUsage();
		}
		else if(strcmp(argv[i], "-device") == 0 && i+1<argc)
		{
			QString str(argv[i+1]);
			bool ok;
			usbDevice = str.toInt(&ok);
			if(!ok)
			{
				usbDevice = -1;
				path = argv[i+1];
			}
			++i;
		}
		else if(strcmp(argv[i], "-width") == 0 && i+1<argc)
		{
			imageWidth = std::atoi(argv[i+1]);
			if(imageWidth < 0)
			{
				showUsage();
			}
			++i;
		}
		else if(strcmp(argv[i], "-height") == 0 && i+1<argc)
		{
			imageHeight = std::atoi(argv[i+1]);
			if(imageHeight < 0)
			{
				showUsage();
			}
			++i;
		}
		else if(strcmp(argv[i], "-fps") == 0 && i+1<argc)
		{
			imageRate = std::atof(argv[i+1]);
			if(imageRate < 0)
			{
				showUsage();
			}
			++i;
		}
		else if(strcmp(argv[i], "-hide") == 0)
		{
			show = false;
		}
		else if(strcmp(argv[i], "-saveto") == 0 && i+1<argc)
		{
			targetDirectory = argv[i+1];
			++i;
		}
		else if(strcmp(argv[i], "-debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
			ULogger::setType(ULogger::kTypeConsole);
		}
		else if(strcmp(argv[i], "-saveext") == 0 && i+1<argc)
		{
			extension = argv[i+1];
			++i;
		}
	}

	printf("Parameters:\n");
	if(usbDevice >= 0)
	{
		printf("   device=%d\n", usbDevice);
	}
	else
	{
		printf("   device (file/directory)=%s\n", path.c_str());
	}
	printf("Parameters:\n"
		   "   width=%d\n"
		   "   height=%d\n"
		   "   hz=%f\n"
		   "   show=%s\n"
		   "   saveto=%s\n"
		   "   saveext=%s\n",
		   imageWidth,
		   imageHeight,
		   imageRate,
		   uBool2Str(show).c_str(),
		   targetDirectory.c_str(),
		   extension.c_str());

	UAbstractImageCapture * capture = 0;
	if(usbDevice>=0)
	{
		// usb device
		capture = new UVideoCapture(usbDevice, imageRate, false, imageWidth, imageHeight);
		printf("Using usb camera %d\n", usbDevice);
	}
	else if(UDirectory::exists(path))
	{
		// images
		capture = new UImageFolderCapture(path, 1, false, imageRate, false, imageWidth, imageHeight);
		printf("Using images from %s\n", path.c_str());
	}
	else
	{
		// video file
		capture = new UVideoCapture(path, imageRate, false, imageWidth, imageHeight);
		printf("Using video file %s\n", path.c_str());

	}
	if(!capture->init())
	{
		printf("Can't initialize the camera...\n");
		delete capture;
		return 1;
	}

	ImageHandler imgHandler(targetDirectory, extension, show, imageRate);
	UEventsManager::addHandler(&imgHandler);

	capture->start();

#ifndef WIN32
	// Catch ctrl-c to close the gui
	// (Place this after QApplication's constructor)
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
#endif

	app.exec();
	UEventsManager::removeHandler(&imgHandler);
	delete capture;
	return 0;
}
