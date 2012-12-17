/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of utilite.
 *
 * utilite is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * utilite is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with utilite.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "utilite/UtiLiteExp.h" // DLL export/import defines

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
class UTILITE_EXP UAbstractImageCapture :
	public UThreadNode
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



/////////////////////////
// UImageFolderCapture
/////////////////////////
class UTILITE_EXP UImageFolderCapture :
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
	UDirectory _dir;
	int _count;
	std::string _lastFileName;



};




/////////////////////////
// UVideoCapture
/////////////////////////
class UTILITE_EXP UVideoCapture :
	public UAbstractImageCapture
{
public:
	enum Source{kVideoFile, kUsbDevice};

public:
	UVideoCapture(int usbDevice = 0,
			float imageRate = 0,
			bool autoRestart = false,
			unsigned int imageWidth = 0,
			unsigned int imageHeight = 0,
			unsigned int framesDropped = 0,
			int id = 0);
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
