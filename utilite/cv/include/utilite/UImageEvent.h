/*
 * UImageEvent.h
 *
 *  Created on: 2012-12-13
 *      Author: mathieu
 */

#ifndef UIMAGEEVENT_H_
#define UIMAGEEVENT_H_

#include <opencv2/core/core.hpp>
#include <utilite/UEvent.h>

class UImageEvent :
	public UEvent
{
public:
	enum Code {
		kCodeImage,
		kCodeNoMoreImages
	};

public:
	UImageEvent(const cv::Mat & image, int cameraId = 0) :
		UEvent(kCodeImage),
		_cameraId(cameraId),
		_image(image)
	{
	}
	UImageEvent(int cameraId = 0) :
		UEvent(kCodeNoMoreImages),
		_cameraId(cameraId)
	{
	}

	int cameraId() const {return _cameraId;}

	// Image or descriptors
	const cv::Mat & image() const {return _image;}

	virtual ~UImageEvent() {}
	virtual std::string getClassName() const {return std::string("UImageEvent");}

private:
	int _cameraId;
	cv::Mat _image;
};

#endif /* UIMAGEEVENT_H_ */
