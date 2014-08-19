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

#include "utilite/UImageCapture.h"
#include "utilite/UEventsManager.h"
#include "utilite/UConversion.h"
#include "utilite/UStl.h"
#include "utilite/UConversion.h"
#include "utilite/UFile.h"
#include "utilite/UDirectory.h"
#include "utilite/UTimer.h"
#include <opencv2/imgproc/imgproc.hpp>

UAbstractImageCapture::UAbstractImageCapture(float imageRate,
		bool autoRestart,
		unsigned int imageWidth,
		unsigned int imageHeight,
		unsigned int framesDropped,
		int id) :
	_imageRate(imageRate),
	_id(id),
	_autoRestart(autoRestart),
	_imageWidth(imageWidth),
	_imageHeight(imageHeight),
	_framesDropped(framesDropped)
{
}

UAbstractImageCapture::~UAbstractImageCapture()
{
	join(true);
}

void UAbstractImageCapture::setImageSize(unsigned int width, unsigned int height)
{
	_imageSizeMutex.lock();
	{
		_imageWidth = width;
		_imageHeight = height;
	}
	_imageSizeMutex.unlock();
}

void UAbstractImageCapture::getImageSize(unsigned int & width, unsigned int & height)
{
	_imageSizeMutex.lock();
	{
		width = _imageWidth;
		height = _imageHeight;
	}
	_imageSizeMutex.unlock();
}

void UAbstractImageCapture::mainLoopBegin()
{
	_frameRateTimer.start();
}

void UAbstractImageCapture::mainLoop()
{
	process();
}

cv::Mat UAbstractImageCapture::takeImage()
{
	float imageRate = _imageRate;
	if(imageRate>0)
	{
		int sleepTime = (1000.0f/imageRate - 1000.0f*_frameRateTimer.getElapsedTime());
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}

		// Add precision at the cost of a small overhead
		while(_frameRateTimer.getElapsedTime() < 1.0/double(imageRate)-0.000001)
		{
			//
		}

		double slept = _frameRateTimer.getElapsedTime();
		_frameRateTimer.start();
		UDEBUG("slept=%fs vs target=%fs", slept, 1.0/double(imageRate));
	}

	cv::Mat img;
	if(!this->isKilled())
	{
		UTimer timer;
		img = this->captureImage();
		UDEBUG("Time capturing image = %fs", timer.ticks());

		if(!img.empty())
		{
			if(img.depth() != CV_8U)
			{
				UWARN("Images should have already 8U depth !?");
				cv::Mat tmp = img;
				img = cv::Mat();
				tmp.convertTo(img, CV_8U);
				UDEBUG("Time converting image to 8U = %fs", timer.ticks());
			}

			if(_framesDropped)
			{
				unsigned int count = 0;
				while(count++ < _framesDropped)
				{
					cv::Mat tmp = this->captureImage();
					if(!tmp.empty())
					{
						UDEBUG("frame dropped (%d/%d)", (int)count, (int)_framesDropped);
					}
					else
					{
						break;
					}
				}
				UDEBUG("Frames dropped time = %fs", timer.ticks());
			}
		}
	}
	return img;
}

void UAbstractImageCapture::process()
{
	UTimer timer;
	ULOGGER_DEBUG("Camera::process()");
	cv::Mat img = this->takeImage();
	if(!img.empty())
	{
		UEventsManager::post(new UImageEvent(img, _id));
	}
	else if(!this->isKilled())
	{
		if(_autoRestart)
		{
			this->init();
		}
		else
		{
			ULOGGER_DEBUG("Camera::process() : no more images...");
			this->kill();
			UEventsManager::post(new UImageEvent(_id));
		}
	}
}


/////////////////////////
// UImageFolderCapture
/////////////////////////
UImageFolderCapture::UImageFolderCapture(const std::string & path,
					 int startAt,
					 bool refreshDir,
					 float imageRate,
					 bool autoRestart,
					 unsigned int imageWidth,
					 unsigned int imageHeight,
					 unsigned int framesDropped,
					 int id) :
	UAbstractImageCapture(imageRate, autoRestart, imageWidth, imageHeight, framesDropped, id),
	_path(path),
	_startAt(startAt),
	_refreshDir(refreshDir),
	_count(0)
{
}

UImageFolderCapture::~UImageFolderCapture(void)
{
	join(true);
}

bool UImageFolderCapture::init()
{
	UDEBUG("");
	_dir = UDir(_path, "jpg ppm png bmp pnm");
	_count = 0;
	if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
	{
		_path.append("/");
	}
	if(!_dir.isValid())
	{
		ULOGGER_ERROR("Directory path is not valid \"%s\"", _path.c_str());
	}
	else if(_dir.getFileNames().size() == 0)
	{
		UWARN("Directory is empty \"%s\"", _path.c_str());
	}
	return _dir.isValid();
}

cv::Mat UImageFolderCapture::captureImage()
{
	UDEBUG("");
	cv::Mat img;
	if(_dir.isValid())
	{
		if(_refreshDir)
		{
			_dir.update();
		}
		if(_startAt == 0)
		{
			const std::list<std::string> & fileNames = _dir.getFileNames();
			if(fileNames.size())
			{
				if(_lastFileName.empty() || uStrNumCmp(_lastFileName,*fileNames.rbegin()) < 0)
				{
					_lastFileName = *fileNames.rbegin();
					std::string fullPath = _path + _lastFileName;
					img = cv::imread(fullPath.c_str());
				}
			}
		}
		else
		{
			std::string fileName;
			std::string fullPath;
			fileName = _dir.getNextFileName();
			if(fileName.size())
			{
				fullPath = _path + fileName;
				while(++_count < _startAt && (fileName = _dir.getNextFileName()).size())
				{
					fullPath = _path + fileName;
				}
				if(fileName.size())
				{
					ULOGGER_DEBUG("Loading image : %s", fullPath.c_str());
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
					img = cv::imread(fullPath.c_str(), cv::IMREAD_UNCHANGED);
#else
					img = cv::imread(fullPath.c_str(), -1);
#endif
					UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d", img.cols, img.rows, img.channels(), img.elemSize(), img.total());

					// FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
					if(img.depth() != CV_8U)
					{
						// The depth should be 8U
						UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
						IplImage * i = cvLoadImage(fullPath.c_str());
						img = cv::Mat(i, true);
						cvReleaseImage(&i);
					}
				}
			}
		}
	}
	else
	{
		UWARN("Directory is not set, camera must be initialized.");
	}

	unsigned int w;
	unsigned int h;
	this->getImageSize(w, h);

	if(!img.empty() &&
	   w &&
	   h &&
	   w != (unsigned int)img.cols &&
	   h != (unsigned int)img.rows)
	{
		cv::Mat resampled;
		cv::resize(img, resampled, cv::Size(w, h));
		img = resampled;
	}

	return img;
}



/////////////////////////
// UVideoCapture
/////////////////////////
UVideoCapture::UVideoCapture(int usbDevice,
						 float imageRate,
						 bool autoRestart,
						 unsigned int imageWidth,
						 unsigned int imageHeight,
						 unsigned int framesDropped,
						 int id) :
	UAbstractImageCapture(imageRate, autoRestart, imageWidth, imageHeight, framesDropped, id),
	_src(kUsbDevice),
	_usbDevice(usbDevice)
{

}

UVideoCapture::UVideoCapture(const std::string & filePath,
						   float imageRate,
						   bool autoRestart,
						   unsigned int imageWidth,
						   unsigned int imageHeight,
						   unsigned int framesDropped,
						   int id) :
	UAbstractImageCapture(imageRate, autoRestart, imageWidth, imageHeight, framesDropped, id),
	_filePath(filePath),
	_src(kVideoFile),
	_usbDevice(0)
{
}

UVideoCapture::~UVideoCapture()
{
	join(true);
	_capture.release();
}

bool UVideoCapture::init()
{
	if(_capture.isOpened())
	{
		_capture.release();
	}

	if(_src == kUsbDevice)
	{
		unsigned int w;
		unsigned int h;
		this->getImageSize(w, h);

		ULOGGER_DEBUG("CameraVideo::init() Usb device initialization on device %d with imgSize=[%d,%d]", _usbDevice, w, h);
		_capture.open(_usbDevice);

		if(w && h)
		{
			_capture.set(CV_CAP_PROP_FRAME_WIDTH, double(w));
			_capture.set(CV_CAP_PROP_FRAME_HEIGHT, double(h));
		}
	}
	else if(_src == kVideoFile)
	{
		ULOGGER_DEBUG("Camera: filename=\"%s\"", _filePath.c_str());
		_capture.open(_filePath.c_str());
	}
	else
	{
		ULOGGER_ERROR("Camera: Unknown source...");
	}
	if(!_capture.isOpened())
	{
		ULOGGER_ERROR("Camera: Failed to create a capture object!");
		_capture.release();
		return false;
	}
	return true;
}

cv::Mat UVideoCapture::captureImage()
{
	cv::Mat img;  // Null image
	if(_capture.isOpened())
	{
		_capture.read(img);
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}

	unsigned int w;
	unsigned int h;
	this->getImageSize(w, h);

	if(!img.empty() &&
	   w &&
	   h &&
	   w != (unsigned int)img.cols &&
	   h != (unsigned int)img.rows)
	{
		cv::Mat resampled;
		cv::resize(img, resampled, cv::Size(w, h));
		return resampled;
	}
	else
	{
		// clone required
		return img.clone();
	}
}
