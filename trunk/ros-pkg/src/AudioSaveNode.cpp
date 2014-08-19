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

#include <ros/ros.h>
#include <utilite/UWav.h>
#include <utilite/UMp3Encoder.h>
#include <utilite/UFile.h>
#include "utilite/AudioFrame.h"

#include <dynamic_reconfigure/server.h>
#include <utilite/AudioSaveConfig.h>

#define FILE_NAME_PREFIX "audio_saved"
bool encodeToMp3 = true;
FILE * fp = 0;
bool initialized = false;
unsigned int dataLength = 0;

int channels = 0;
int bits = 0;
float rate = 0;

void frameReceivedCallback(const utilite::AudioFramePtr & msg)
{
	if(fp)
	{
		if(!initialized)
		{
			channels = msg->nChannels;
			bits = msg->sampleSize*8;
			rate = msg->fs;
			UWav::writeWavHeader(fp, dataLength, rate, channels, bits);
			initialized = true;
			ROS_INFO("Stream received with channels=%d, bits/sample=%d, rate=%f", channels, bits, rate);
		}
		ROS_INFO("Received frame (%fs)...", (float(dataLength)/float(bits/8))/rate);
	}

	dataLength += fwrite(msg->data.data(), 1, msg->data.size(), fp);
}

void callback(utilite::AudioSaveConfig &config, uint32_t level)
{
	encodeToMp3 = config.encode_to_mp3;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "audio_save");

	std::string fileName = FILE_NAME_PREFIX ".wav";
	fp = fopen(fileName.c_str(), "wb");

	if(!fp)
	{
		ROS_ERROR("Cannot open file %s for writing.", fileName.c_str());
		return 1;
	}

	dynamic_reconfigure::Server<utilite::AudioSaveConfig> server;
	dynamic_reconfigure::Server<utilite::AudioSaveConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::NodeHandle nh;
	ros::Subscriber audioSubs = nh.subscribe("audioFrame", 0, frameReceivedCallback);

	ROS_INFO("Created file %s (might be located in ~/.ros if roslaunch is used).", fileName.c_str());
	ROS_INFO("Waiting for stream...");
	ros::spin();

	//finalize
	if(fp && initialized)
	{
		printf("\nFinalizing header (data length = %d bytes)...", dataLength);
		// Write back the wav header now that we know its length.
		UWav::writeWavHeader(fp, dataLength, rate, channels, bits);
		fclose(fp);
		fp = 0;

		if(encodeToMp3)
		{
			// Encode to mp3
			UMp3Encoder mp3Encoder;
			std::string fileNameMp3 = FILE_NAME_PREFIX ".mp3";
			printf("\nEncoding to mp3 (%s)...", fileNameMp3.c_str());
			if(mp3Encoder.encode(fileName, fileNameMp3) == 0)
			{
				//Erase the wav file
				UFile::erase(fileName);
			}
		}
	}
	else if(fp)
	{
		fclose(fp);
		fp = 0;
	}
	printf("\nDone! Closing...\n");

	return 0;
}
