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
