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

#pragma once

#include "utilite/UtiLiteCvExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <utilite/UThreadNode.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEvent.h>
#include <utilite/UDirectory.h>
#include <utilite/UTimer.h>
#include <utilite/UImageEvent.h>
#include <set>
#include <stack>
#include <list>
#include <vector>

/**
 * Class UAbstractImageCapture
 *
 */
class UTILITECV_EXP UAbstractImageCapture :
	public UThread
{
public:
	virtual ~UAbstractImageCapture();
	cv::Mat takeImage();
	virtual bool init() = 0;

	//getters
	bool isPaused() const {return !this->isRunning();}
	bool isCapturing() const {return this->isRunning();}
	void getImageSize(unsigned int & width, unsigned int & height);
	float getImageRate() const {return _imageRate;}

	//setters
	void setImageRate(float imageRate) {_imageRate = imageRate;}
	void setAutoRestart(bool autoRestart) {_autoRestart = autoRestart;}
	void setImageSize(unsigned int width, unsigned int height);


	int id() const {return _id;}

protected:
	/**
	 * Constructor
	 *
	 * @param imageRate : image/second , 0 for fast as the camera can
	 */
	UAbstractImageCapture(float imageRate = 0, bool autoRestart = false, unsigned int imageWidth = 0, unsigned int imageHeight = 0, unsigned int framesDropped = 0, int id = 0);

	virtual cv::Mat captureImage() = 0;

private:
	virtual void mainLoopBegin();
	virtual void mainLoop();
	void process();

private:
	float _imageRate;
	int _id;
	bool _autoRestart;
	unsigned int _imageWidth;
	unsigned int _imageHeight;
	unsigned int _framesDropped;
	UTimer _frameRateTimer;
	UMutex _imageSizeMutex;
};



/**
 * UImageFolderCapture : Read images from a directory.
 */
class UTILITECV_EXP UImageFolderCapture :
	public UAbstractImageCapture
{
public:
	UImageFolderCapture(const std::string & path,
			int startAt = 1,
			bool refreshDir = false,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0,
			int id = 0);
	virtual ~UImageFolderCapture();

	virtual bool init();
	std::string getPath() const {return _path;}

protected:
	virtual cv::Mat captureImage();

private:
	std::string _path;
	int _startAt;
	// If the list of files in the directory is refreshed
	// on each call of takeImage()
	bool _refreshDir;
	UDir _dir;
	int _count;
	std::string _lastFileName;



};




/**
 * UVideoCapture : Read images from a usb device (webcam) or a video file.
 */
class UTILITECV_EXP UVideoCapture :
	public UAbstractImageCapture
{
public:
	enum Source{kVideoFile, kUsbDevice};

public:
	/**
	 * Usb device constructor
	 */
	UVideoCapture(int usbDevice = 0,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0,
			int id = 0);
	/**
	 * Video file constructor
	 */
	UVideoCapture(const std::string & filePath,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0,
			int id = 0);
	virtual ~UVideoCapture();

	virtual bool init();
	int getUsbDevice() const {return _usbDevice;}
	const std::string & getFilePath() const {return _filePath;}

protected:
	virtual cv::Mat captureImage();

private:
	// File type
	std::string _filePath;

	cv::VideoCapture _capture;
	Source _src;

	// Usb camera
	int _usbDevice;
};
