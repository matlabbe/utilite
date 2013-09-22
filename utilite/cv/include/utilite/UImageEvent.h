/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UIMAGEEVENT_H_
#define UIMAGEEVENT_H_

#include <opencv2/core/core.hpp>
#include <utilite/UEvent.h>

/**
 * An image event.
 */
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
