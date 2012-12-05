/*
 * AudioOverlapNode.cpp
 */

#include <ros/ros.h>

#include <utilite/ULogger.h>
#include <rtabmap/core/Micro.h>

#include "rtabmap_audio/AudioFrame.h"
#include "rtabmap_audio/AudioFrameFreq.h"
#include "rtabmap_audio/AudioFrameFreqSqrdMagn.h"

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
		audioFramePublisher_ = nh.advertise<rtabmap_audio::AudioFrame>("audioOverlappedFrame", 1);
		audioFrameFreqPublisher_ = nh.advertise<rtabmap_audio::AudioFrameFreq>("audioOverlappedFrameFreq", 1);
		audioFrameFreqSqrdMagnPublisher_ = nh.advertise<rtabmap_audio::AudioFrameFreqSqrdMagn>("audioOverlappedFrameFreqSqrdMagn", 1);
	}
	~AudioOverlap()
	{
		if(micro_)
		{
			delete micro_;
		}
	}

private:
	void frameReceivedCallback(const rtabmap_audio::AudioFramePtr & msgReceived)
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
			ROS_ERROR("frame bytes size (%d) != specifications (frameLength=%d * nChannels=%d * sampleSize=%d)", msgReceived->data.size(), msgReceived->frameLength, msgReceived->nChannels, msgReceived->sampleSize);
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
			ROS_INFO("buffer.frame.size()=%d, frameLength = %d",buffer.frame.size(), buffer.frame.size() / (msgReceived->nChannels * msgReceived->sampleSize));
			micro_ = new rtabmap::Micro(
				rtabmap::MicroEvent::kTypeFrame,
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
				rtabmap_audio::AudioFramePtr msg(new rtabmap_audio::AudioFrame);
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
				cv::Mat data(msgReceived->nChannels, frameLength, msgReceived->sampleSize==1?CV_8S:msgReceived->sampleSize==2?CV_16S:CV_32S);
				// "uninterleave" the data
				for(unsigned int i=0; i<buffer.frame.size(); i+=data.elemSize()*data.rows)
				{
					for(int j=0; j<data.rows; ++j)
					{
						memcpy(data.data + (i/(data.elemSize()*data.rows))*data.elemSize() + j*data.cols*data.elemSize(), buffer.frame.data()+i+j*data.elemSize(), data.elemSize());
					}
				}

				cv::Mat freq;
				freq = micro_->computeFFT(data, false);

				if(audioFrameFreqPublisher_.getNumSubscribers())
				{
					rtabmap_audio::AudioFrameFreqPtr msg(new rtabmap_audio::AudioFrameFreq);
					msg->header.frame_id = msgReceived->header.frame_id;
					msg->header.stamp = msgReceived->header.stamp;
					msg->data.resize(freq.total()*freq.elemSize());
					memcpy(msg->data.data(), freq.data, msg->data.size());
					msg->frameLength = freq.cols;
					msg->fs = msgReceived->fs;
					msg->nChannels = freq.rows;
					audioFrameFreqPublisher_.publish(msg);
				}
				if(audioFrameFreqSqrdMagnPublisher_.getNumSubscribers())
				{
					rtabmap_audio::AudioFrameFreqSqrdMagnPtr msg(new rtabmap_audio::AudioFrameFreqSqrdMagn);
					msg->header.frame_id = msgReceived->header.frame_id;
					msg->header.stamp = msgReceived->header.stamp;

					//compute the squared magnitude
					cv::Mat sqrdMagn(freq.rows, freq.cols/2, CV_32F);
					//for each channels
					for(int i=0; i<sqrdMagn.rows; ++i)
					{
						cv::Mat rowFreq = freq.row(i);
						cv::Mat rowSqrdMagn = sqrdMagn.row(i);
						float re;
						float im;
						for(int j=0; j<rowSqrdMagn.cols; ++j)
						{
							re = rowFreq.at<float>(0, j*2);
							im = rowFreq.at<float>(0, j*2+1);
							rowSqrdMagn.at<float>(0, j) = re*re + im*im;
						}
					}

					msg->data.resize(sqrdMagn.cols * sqrdMagn.rows);
					memcpy(msg->data.data(), sqrdMagn.data, sqrdMagn.total() * sqrdMagn.elemSize());
					msg->frameLength = sqrdMagn.cols;
					msg->fs = msgReceived->fs;
					msg->nChannels = sqrdMagn.rows;

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
	rtabmap::Micro * micro_;
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
