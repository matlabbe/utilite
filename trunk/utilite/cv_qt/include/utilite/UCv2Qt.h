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

#ifndef UCV2QT_H_
#define UCV2QT_H_

#include <QtGui/QImage>
#include <opencv2/core/core.hpp>
#include <utilite/UMath.h>
#include <utilite/UThread.h>

/**
 * Convert a cv::Mat image to a QImage. Support 
 * depth (float32, uint16) image and RGB/BGR 8bits images.
 * @param image the cv::Mat image (can be 1 channel [CV_8U, CV_16U or CV_32F] or 3 channels [CV_U8])
 * @param isBgr if 3 channels, it is BGR or RGB order.
 * @return the QImage
 */
inline QImage uCvMat2QImage(const cv::Mat & image, bool isBgr = true)
{
	QImage qtemp;
	if(!image.empty() && image.depth() == CV_8U)
	{
		if(image.channels()==3)
		{
			const unsigned char * data = image.data;
			if(image.channels() == 3)
			{
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
		}
		else if(image.channels() == 1)
		{
			// mono grayscale
			qtemp = QImage(image.data, image.cols, image.rows, image.cols, QImage::Format_Indexed8).copy();
		}
		else
		{
			printf("Wrong image format, must have 1 or 3 channels\n");
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
				if(data[x] < min || data[x] > max || uIsNan(data[x]))
				{
					*p = 0;
				}
				else
				{
					*p = uchar(255.0f - ((data[x]-min)*255.0f)/(max-min));
					if(*p == 255)
					{
						*p = 0;
					}
				}
			}
		}

		QVector<QRgb> my_table;
		for(int i = 0; i < 256; i++) my_table.push_back(qRgb(i,i,i));
		qtemp.setColorTable(my_table);
    }
	else if(image.depth() == CV_16U && image.channels()==1)
	{
		// Assume depth image (unsigned short in mm)
		const unsigned short * data = (const unsigned short *)image.data;
		unsigned short min=data[0], max=data[0];
		for(unsigned int i=1; i<image.total(); ++i)
		{
			if(!uIsNan(data[i]) && data[i] > 0)
			{
				if((uIsNan(min) && data[i] > 0) ||
				   (data[i] > 0 && data[i]<min))
				{
					min = data[i];
				}
				if((uIsNan(max) && data[i] > 0) ||
				   (data[i] > 0 && data[i]>max))
				{
					max = data[i];
				}
			}
		}

		qtemp = QImage(image.cols, image.rows, QImage::Format_Indexed8);
		for(int y = 0; y < image.rows; ++y, data += image.cols)
		{
			for(int x = 0; x < image.cols; ++x)
			{
				uchar * p = qtemp.scanLine (y) + x;
				if(data[x] < min || data[x] > max || uIsNan(data[x]) || max == min)
				{
					*p = 0;
				}
				else
				{
					*p = uchar(255.0f - (float(data[x]-min)/float(max-min))*255.0f);
					if(*p == 255)
					{
						*p = 0;
					}
				}
			}
		}

		QVector<QRgb> my_table;
		for(int i = 0; i < 256; i++) my_table.push_back(qRgb(i,i,i));
		qtemp.setColorTable(my_table);
	}
	else if(!image.empty() && image.depth() != CV_8U)
	{
		printf("Wrong image format, must be 8_bits/3channels or (depth) 32bitsFloat/1channel, 16bits/1channel\n");
	}
	return qtemp;
}

class UCvMat2QImageThread : public UThread
{
public:
	UCvMat2QImageThread(const cv::Mat & image, bool isBgr = true) :
		image_(image),
		isBgr_(isBgr) {}
	QImage & getQImage() {return qtImage_;}
protected:
	virtual void mainLoop()
	{
		qtImage_ = uCvMat2QImage(image_, isBgr_);
		this->kill();
	}
private:
	cv::Mat image_;
	bool isBgr_;
	QImage qtImage_;
};

#endif /* UCV2QT_H_ */
