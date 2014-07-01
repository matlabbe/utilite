/*
 * AudioOverlapNode.cpp
 */

#include <ros/ros.h>

#include <utilite/ULogger.h>
#include <utilite/UAudioCaptureFFT.h>

#include "utilite/AudioFrame.h"
#include "utilite/AudioFrameFreq.h"
#include "utilite/AudioFrameFreqSqrdMagn.h"

class FrameBuffer
{
public:
	FrameBuffer() :
		fs(0),
		frameLength(0),
		nChannels(0),
		sampleSize(0),
		overlapSize(0),
		ready(false)
	{}
	std::vector<uint8_t> frame; //interleaved data
	uint32_t fs;
	uint32_t frameLength;
	uint32_t nChannels;
	uint32_t sampleSize;
	int overlapSize;
	bool ready;
};

FrameBuffer buffer;

class AudioOverlap
{
public:
	AudioOverlap(double ratio) :
		ratio_(ratio),
		micro_(0)
	{
		ros::NodeHandle nh;
		audioSubs_ = nh.subscribe("audioFrame", 1, &AudioOverlap::frameReceivedCallback, this);
		audioFramePublisher_ = nh.advertise<utilite::AudioFrame>("audioOverlappedFrame", 1);
		audioFrameFreqPublisher_ = nh.advertise<utilite::AudioFrameFreq>("audioOverlappedFrameFreq", 1);
		audioFrameFreqSqrdMagnPublisher_ = nh.advertise<utilite::AudioFrameFreqSqrdMagn>("audioOverlappedFrameFreqSqrdMagn", 1);
	}
	~AudioOverlap()
	{
		if(micro_)
		{
			delete micro_;
		}
	}

private:
	void frameReceivedCallback(const utilite::AudioFramePtr & msgReceived)
	{
		if(!msgReceived->data.size())
		{
			ROS_WARN("Frame empty?!?");
			return;
		}
		else if(msgReceived->sampleSize!=1 && msgReceived->sampleSize!=2 && msgReceived->sampleSize!=4)
		{
			ROS_WARN("frame sample size must be 1, 2 or 4 bytes/sample (received=%d)", msgReceived->sampleSize);
			return;
		}
		else if(msgReceived->data.size() != msgReceived->frameLength * msgReceived->nChannels * msgReceived->sampleSize)
		{
			ROS_ERROR("frame bytes size (%d) != specifications (frameLength=%d * nChannels=%d * sampleSize=%d)", (int)msgReceived->data.size(), msgReceived->frameLength, msgReceived->nChannels, msgReceived->sampleSize);
			return;
		}

		if(micro_ && (buffer.fs != msgReceived->fs || buffer.frameLength != msgReceived->frameLength || buffer.nChannels != msgReceived->nChannels || buffer.sampleSize != msgReceived->sampleSize))
		{
			ROS_WARN("Frame size changed, reinitializing...");
			delete micro_;
			micro_ = 0;
			buffer = FrameBuffer();
		}

		if(!micro_)
		{
			buffer.overlapSize = int(float(msgReceived->data.size()) * ratio_);
			if(buffer.overlapSize % (msgReceived->nChannels*msgReceived->sampleSize) != 0)
			{
				int dif = buffer.overlapSize - (buffer.overlapSize/(msgReceived->nChannels*msgReceived->sampleSize))*msgReceived->nChannels*msgReceived->sampleSize;
				buffer.overlapSize -= dif;
			}
			if(buffer.overlapSize <= 0)
			{
				buffer.overlapSize = 0;
				buffer.ready = true;
			}

			buffer.frame.resize(msgReceived->data.size() + buffer.overlapSize);
			buffer.fs = msgReceived->fs;
			buffer.frameLength = msgReceived->frameLength;
			buffer.nChannels = msgReceived->nChannels;
			buffer.sampleSize = msgReceived->sampleSize;

			// this should initialize FFTW stuff (to use Micro::computeFFT()) and
			// no need to call micro_->init()
			ROS_INFO("buffer.frame.size()=%d, frameLength = %d", (int)buffer.frame.size(), (int)buffer.frame.size() / (msgReceived->nChannels * msgReceived->sampleSize));
			micro_ = new UAudioCaptureFFT(
				UAudioEvent::kTypeFrame,
				0, msgReceived->fs,
				buffer.frame.size() / (msgReceived->nChannels * msgReceived->sampleSize),
				msgReceived->nChannels,
				msgReceived->sampleSize);
		}
		else
		{
			buffer.ready = true;
		}

		memcpy(buffer.frame.data(), buffer.frame.data()+msgReceived->data.size(), buffer.overlapSize);
		memcpy(buffer.frame.data()+buffer.overlapSize, msgReceived->data.data(), msgReceived->data.size());

		if(buffer.ready)
		{
			if(audioFramePublisher_.getNumSubscribers())
			{
				utilite::AudioFramePtr msg(new utilite::AudioFrame);
				msg->header.frame_id = msgReceived->header.frame_id;
				msg->header.stamp = msgReceived->header.stamp;
				msg->data = buffer.frame;
				msg->frameLength = msgReceived->frameLength;
				msg->fs = msgReceived->fs;
				msg->nChannels = msgReceived->nChannels;
				msg->sampleSize = msgReceived->sampleSize;
				audioFramePublisher_.publish(msg);
			}

			if(audioFrameFreqPublisher_.getNumSubscribers() || audioFrameFreqSqrdMagnPublisher_.getNumSubscribers())
			{
				int frameLength = buffer.frame.size() / (msgReceived->nChannels * msgReceived->sampleSize);
				// "uninterleave" the data
				std::vector<std::vector<char> > data(msgReceived->nChannels, std::vector<char>(frameLength*msgReceived->sampleSize));
				for(unsigned int i = 0; i<buffer.frame.size(); i+=msgReceived->nChannels*msgReceived->sampleSize)
				{
					for(unsigned int j=0; j<data.size(); ++j)
					{
						memcpy(data[j].data() + i/data.size(), buffer.frame.data() + i + j*msgReceived->sampleSize, msgReceived->sampleSize);
					}
				}

				std::vector<std::vector<float> > freq;
				freq = micro_->computeFFT(data, false);

				if(audioFrameFreqPublisher_.getNumSubscribers())
				{
					utilite::AudioFrameFreqPtr msg(new utilite::AudioFrameFreq);
					msg->header.frame_id = msgReceived->header.frame_id;
					msg->header.stamp = msgReceived->header.stamp;
					msg->data.resize(freq.size()*freq[0].size());
					for(unsigned int i=0; i<freq.size(); ++i)
					{
						memcpy(msg->data.data()+i*freq[i].size(), freq[i].data(), freq[i].size()*sizeof(float));
					}
					msg->frameLength = freq[0].size();
					msg->fs = msgReceived->fs;
					msg->nChannels = freq.size();
					audioFrameFreqPublisher_.publish(msg);
				}
				if(audioFrameFreqSqrdMagnPublisher_.getNumSubscribers())
				{
					utilite::AudioFrameFreqSqrdMagnPtr msg(new utilite::AudioFrameFreqSqrdMagn);
					msg->header.frame_id = msgReceived->header.frame_id;
					msg->header.stamp = msgReceived->header.stamp;

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
					msg->fs = msgReceived->fs;
					msg->nChannels = freq.size();

					audioFrameFreqSqrdMagnPublisher_.publish(msg);
				}
			}
		}
	}
private:
	ros::Subscriber audioSubs_;
	ros::Publisher audioFramePublisher_;
	ros::Publisher audioFrameFreqPublisher_;
	ros::Publisher audioFrameFreqSqrdMagnPublisher_;
	double ratio_;
	UAudioCaptureFFT * micro_;
};


int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
	ros::init(argc, argv, "audioOverlap");

	ros::NodeHandle nh("~");
	double ratio = 0.5;
	nh.param("ratio", ratio, ratio);

	AudioOverlap ao(ratio);

	ros::spin();

	return 0;
}
