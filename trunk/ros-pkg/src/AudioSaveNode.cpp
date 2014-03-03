/*
 * AudioSaveNode.cpp
 */

#include <ros/ros.h>
#include <utilite/UWav.h>
#include <utilite/UMp3Encoder.h>
#include <utilite/UFile.h>
#include "utilite/AudioFrame.h"

#include <dynamic_reconfigure/server.h>
#include <utilite/audioSaveConfig.h>

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

void callback(utilite::audioSaveConfig &config, uint32_t level)
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

	dynamic_reconfigure::Server<utilite::audioSaveConfig> server;
	dynamic_reconfigure::Server<utilite::audioSaveConfig>::CallbackType f;
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
