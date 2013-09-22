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

#ifndef UCV2QT_H_
#define UCV2QT_H_

#include <QtGui/QImage>
#include <opencv2/core/core.hpp>

/**
 * Convert a cv::Mat image to a QImage.
 * @param image the cv::Mat image (can be 1 channel or 3 channels of type CV_U8)
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
	else if(!image.empty() && image.depth() != CV_8U)
	{
		printf("Wrong image format, must be 8_bits, 3 channels\n");
	}
	return qtemp;
}


#endif /* UCV2QT_H_ */
