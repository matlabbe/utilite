/*
 * imagesJoiner.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: mathieu
 */

#include "utilite/ULogger.h"
#include "utilite/UTimer.h"
#include "utilite/UDirectory.h"
#include "utilite/UFile.h"
#include "utilite/UConversion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void showUsage()
{
	printf("Usage:\n"
			"imagesJoiner.exe image1.png image2.png image3.png ...\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	std::vector<cv::Mat> images;
	std::string path;
	for(int i=1; i<argc; ++i)
	{
		cv::Mat image = cv::imread(argv[i]);
		if(!image.empty())
		{
			if(images.size())
			{
				UASSERT(image.cols == images[0].cols &&
						image.rows == images[0].rows &&
						image.type() == images[0].type());
			}
			else
			{
				path = argv[i];
			}
			images.push_back(image);
		}
		else
		{
			UERROR("Cannot read image \"%s\"", argv[i]);
		}
	}
	UASSERT(images.size() > 1);

	std::string outputPath = UDir::getDir(path) + UDir::separator() + UFile::getName(path, false) + "-merged." + UFile::getExtension(path);

	cv::Mat output(images[0].rows*images.size(), images[0].cols, images[0].type());

	for(unsigned int i=0; i<images.size(); ++i)
	{
		images[i].copyTo(output.rowRange(i*images[0].rows, (i+1)*images[0].rows));
	}

	if(cv::imwrite(outputPath, output))
	{
		UINFO("Images merged to \"%s\"", outputPath.c_str());
	}
	else
	{
		UERROR("Failed to saved the merged image to \"%s\"", outputPath.c_str());
	}

	return 0;
}


