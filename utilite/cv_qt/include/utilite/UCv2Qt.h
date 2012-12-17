/*
 * UCv2Qt.h
 *
 *  Created on: 2012-12-06
 *      Author: mathieu
 */

#ifndef UCV2QT_H_
#define UCV2QT_H_

#include <QtGui/QImage>
#include <opencv2/core/core.hpp>

// assume bgr
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
	else if(!image.empty() && image.depth() != CV_8U)
	{
		printf("Wrong image format, must be 8_bits, 3 channels\n");
	}
	return qtemp;
}


#endif /* UCV2QT_H_ */
