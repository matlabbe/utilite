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

#include <utilite/ULogger.h>
#include <utilite/UFile.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEventsManager.h>
#include <utilite/UEvent.h>
#include <utilite/UThreadNode.h>

#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#include <utilite/UAudioCaptureFFT.h>

#include "utilite/AudioFrame.h"
#include "utilite/AudioFrameFreq.h"
#include "utilite/AudioFrameFreqSqrdMagn.h"

class EndEvent : public UEvent
{
public:
	virtual std::string getClassName() const {return "EndEvent";}
};

class MicroWrapper : public UThreadNode
{
public:
	MicroWrapper(int deviceId, int frameLength, int fs, int sampleSize, int nChannels) :
		seqNumber_(0)
	{
		micro_ = new UAudioCaptureFFT(UAudioEvent::kTypeFrame, deviceId, fs, frameLength, nChannels, sampleSize);
		ros::NodeHandle nh("");
		audioFramePublisher_ = nh.advertise<utilite::AudioFrame>("audioFrame", 1);
		audioFrameFreqPublisher_ = nh.advertise<utilite::AudioFrameFreq>("audioFrameFreq", 1);
		audioFrameFreqSqrdMagnPublisher_ = nh.advertise<utilite::AudioFrameFreqSqrdMagn>("audioFrameFreqSqrdMagn", 1);

		ros::NodeHandle np("~");
		pauseSrv_ = np.advertiseService("pause", &MicroWrapper::pause, this);
	}

	MicroWrapper(const std::string & fileName, float frameRate, const std::vector<std::string> & nextFileNames) :
		nextFileNames_(nextFileNames)
	{
		micro_ = new UAudioCaptureFFT(UAudioEvent::kTypeFrame, fileName, frameRate);
		ros::NodeHandle nh("");
		audioFramePublisher_ = nh.advertise<utilite::AudioFrame>("audioFrame", 1);
		audioFrameFreqPublisher_ = nh.advertise<utilite::AudioFrameFreq>("audioFrameFreq", 1);
		audioFrameFreqSqrdMagnPublisher_ = nh.advertise<utilite::AudioFrameFreqSqrdMagn>("audioFrameFreqSqrdMagn", 1);

		ros::NodeHandle np("~");
		pauseSrv_ = np.advertiseService("pause", &MicroWrapper::pause, this);
		if(nextFileNames.size())
		{
			nextSrv_ = np.advertiseService("next", &MicroWrapper::next, this);
		}
	}

	virtual ~MicroWrapper()
	{
		this->join(true);
		micro_->join(true);
		delete micro_;
	}

	bool init()
	{
		seqNumber_ = 0;
		return micro_->init();
	}

	unsigned int freq() const
	{
		return micro_->fs();
	}

	unsigned int sampleSize() const
	{
		return micro_->bytesPerSample();
	}

	unsigned int nChannels() const
	{
		return micro_->channels();
	}

	unsigned int samples() const
	{
		return micro_->samples();
	}

	void setPositionMs(int ms)
	{
		micro_->setPositionMs(ms);
	}

	bool pause(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		if(this->isRunning())
		{
			this->kill();
		}
		else
		{
			this->start();
		}
		return true;
	}

	bool next(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		this->join(true);
		this->next();
		this->start();
		return true;
	}

	void next()
	{
		micro_->stop();
		float frameRate = micro_->frameRate();
		std::string currentFileName;
		bool init = false;
		while(!init && nextFileNames_.size())
		{
			delete micro_;
			micro_ = new UAudioCaptureFFT(UAudioEvent::kTypeFrame, nextFileNames_[0], frameRate);
			currentFileName = nextFileNames_[0];
			nextFileNames_.erase(nextFileNames_.begin());
			init = micro_->init();
		}
		if(init)
		{
			ROS_INFO("Recording from file \"%s\"", currentFileName.c_str());
			int soundLengthMs = (this->samples())/(this->nChannels()*this->sampleSize()*(this->freq()/1000));
			int min = (soundLengthMs/1000)/60;
			int sec = (soundLengthMs/1000) - min*60;
			int ms = soundLengthMs - sec*1000 - min*60*1000;
			ROS_INFO("Sound length = %d:%d.%d s", min, sec, ms);
			micro_->startRecorder();
		}
	}

protected:
	virtual void mainLoopBegin()
	{
		if(micro_)
		{
			micro_->startRecorder();
		}
	}

	virtual void mainLoop()
	{
		bool computeFFT = false;
		if(audioFrameFreqPublisher_.getNumSubscribers() || audioFrameFreqSqrdMagnPublisher_.getNumSubscribers())
		{
			computeFFT = true;
		}

		std::vector<std::vector<char> > data;
		std::vector<std::vector<float> > freq;
		if(!computeFFT)
		{
			data = micro_->getFrame();
		}
		else
		{
			data = micro_->getFrame(freq, false);
		}
		if(!data.empty())
		{
			++seqNumber_;
			ros::Time now = ros::Time::now();
			if(audioFramePublisher_.getNumSubscribers())
			{
				utilite::AudioFramePtr msg(new utilite::AudioFrame);
				msg->header.frame_id = "micro";
				msg->header.stamp = now;
				msg->header.seq = seqNumber_;
				// Interleave the data
				int channels = data.size();
				int sampleSize = micro_->bytesPerSample();
				int frameLength = data[0].size()/sampleSize;
				msg->data.resize(frameLength*sampleSize*channels);
				for(unsigned int i=0; i<msg->data.size(); i+=sampleSize*channels)
				{
					for(int j=0; j<channels; ++j)
					{
						memcpy(msg->data.data()+i+j*sampleSize, data[j].data() + i/channels, sampleSize);
					}
				}
				msg->frameLength = frameLength;
				msg->fs = micro_->fs();
				msg->nChannels = channels;
				msg->sampleSize = sampleSize;
				audioFramePublisher_.publish(msg);
			}
			if(!freq.empty())
			{
				if(audioFrameFreqPublisher_.getNumSubscribers())
				{
					utilite::AudioFrameFreqPtr msg(new utilite::AudioFrameFreq);
					msg->header.frame_id = "micro";
					msg->header.stamp = now;
					msg->header.seq = seqNumber_;
					msg->data.resize(freq.size()*freq[0].size());
					for(unsigned int i=0; i<freq.size(); ++i)
					{
						memcpy(msg->data.data()+i*freq[i].size(), freq[i].data(), freq[i].size()*sizeof(float));
					}
					msg->frameLength = freq[0].size();
					msg->fs = micro_->fs();
					msg->nChannels = freq.size();
					audioFrameFreqPublisher_.publish(msg);
				}
				if(audioFrameFreqSqrdMagnPublisher_.getNumSubscribers())
				{
					utilite::AudioFrameFreqSqrdMagnPtr msg(new utilite::AudioFrameFreqSqrdMagn);
					msg->header.frame_id = "micro";
					msg->header.stamp = now;
					msg->header.seq = seqNumber_;

					//compute the squared magnitude
					msg->data.resize(freq.size() * freq[0].size()/2);

					// for each channel
					for(unsigned int j=0; j<freq.size(); ++j)
					{
						std::vector<float> & rowFreq = freq.at(j);

						float re;
						float im;
						for(unsigned int i=0; i<rowFreq.size()/2; ++i)
						{
							re = rowFreq[i*2];
							im = rowFreq[i*2+1];
							msg->data[i+j*rowFreq.size()/2] = re*re+im*im; // squared magnitude
						}
					}
					msg->frameLength = freq[0].size()/2;
					msg->fs = micro_->fs();
					msg->nChannels = freq.size();

					audioFrameFreqSqrdMagnPublisher_.publish(msg);
				}
			}
		}
		else if(nextFileNames_.size())
		{
			this->next();
		}
		else
		{
			UEventsManager::post(new EndEvent());
			this->kill();
		}
	}

private:
	ros::Publisher audioFramePublisher_;
	ros::Publisher audioFrameFreqPublisher_;
	ros::Publisher audioFrameFreqSqrdMagnPublisher_;
	UAudioCaptureFFT * micro_;
	std::vector<std::string> nextFileNames_;
	ros::ServiceServer pauseSrv_;
	ros::ServiceServer nextSrv_;
	int seqNumber_;
};

class Handler: public UEventsHandler
{
public:
	Handler() {UEventsManager::addHandler(this);}
	virtual ~Handler() {UEventsManager::removeHandler(this);}
protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("EndEvent") == 0)
		{
			ROS_INFO("End of stream reached... shutting down!");
			ros::shutdown();
		}
	}
};

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
	ros::init(argc, argv, "audio_recorder");
	ros::NodeHandle nh("~");

	int deviceId = 0;
	std::string path = "";
	int delay = 0;
	int delayms = 0;
	int frameLength = 4800;
	double frameRate = 10.0; // 100 ms frame length
	int fs = 48000;
	int sampleSize = 2;
	int nChannels = 1;

	nh.param("device_id", deviceId, deviceId);
	nh.param("path", path, path);
	nh.param("delay", delay, delay); // sec
	nh.param("delayms", delayms, delayms); // ms
	nh.param("frame_length", frameLength, frameLength);
	nh.param("frame_rate", frameRate, frameRate);
	nh.param("fs", fs, fs);
	nh.param("sample_size", sampleSize, sampleSize);
	nh.param("channels", nChannels, nChannels);

	std::vector<std::string> filePaths; // in case of a directory
	if(UDirectory::exists(path))
	{
		ROS_INFO("path = %s", path.c_str());
		UDirectory dir(path, "wav mp3");
		const std::list<std::string> & fileNames = dir.getFileNames();
		ROS_INFO("files = %d", (int)fileNames.size());
		filePaths.resize(fileNames.size());
		std::list<std::string>::const_iterator iter = fileNames.begin();
		for(unsigned int i=0; i<filePaths.size(); ++i, ++iter)
		{
			filePaths[i] = path + UDirectory::separator() + *iter;
		}
		if(filePaths.size())
		{
			path = filePaths[0];
			filePaths.erase(filePaths.begin());
		}
	}

	MicroWrapper * micro = 0;
	bool fromFile = false;
	if(path.size() && UFile::exists(path))
	{
		ROS_INFO("Recording from file \"%s\"", path.c_str());
		micro = new MicroWrapper(path, frameRate, filePaths);
		ROS_INFO("frame_rate=%f", frameRate);
		fromFile = true;
	}
	else
	{
		ROS_INFO("Recording from microphone device_id=%d", deviceId);
		micro = new MicroWrapper(deviceId, frameLength, fs, sampleSize, nChannels);
		ROS_INFO("frame_length=%d", frameLength);
		ROS_INFO("fs=%d", fs);
		ROS_INFO("sample_size=%d", sampleSize);
		ROS_INFO("channels=%d", nChannels);
	}

	if(!micro->init())
	{
		ROS_ERROR("Cannot initiate the audio recorder.");
	}
	else
	{
		if(fromFile)
		{
			ROS_INFO("fs=%d", micro->freq());
			ROS_INFO("sample_size=%d", micro->sampleSize());
			ROS_INFO("channels=%d", micro->nChannels());
			if(delayms)
			{
				ROS_INFO("delay=%d ms", delayms);
				micro->setPositionMs(delayms);
			}
			else if(delay)
			{
				ROS_INFO("delay=%d sec", delay);
				micro->setPositionMs(delay*1000);
			}
			int soundLengthMs = (micro->samples())/(micro->nChannels()*micro->sampleSize()*(micro->freq()/1000));
			int min = (soundLengthMs/1000)/60;
			int sec = (soundLengthMs/1000) - min*60;
			int ms = soundLengthMs - sec*1000 - min*60*1000;
			ROS_INFO("Sound length = %d:%d.%d s", min, sec, ms);
		}

		Handler h;
		// Start the mic
		micro->start();
		ROS_INFO("Audio recorder started...");
		ros::spin();
	}

	micro->join(true);
	delete micro;

	return 0;
}
