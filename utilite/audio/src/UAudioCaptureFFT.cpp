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

#include "utilite/UAudioCaptureFFT.h"

#include "utilite/UAudioCaptureMic.h"
#include "utilite/UAudioCaptureFile.h"
#include <utilite/UEventsManager.h>
#include <utilite/UFile.h>
#include <utilite/UMath.h>
#include <fftw3.h>
#include <string.h>



UAudioCaptureFFT::UAudioCaptureFFT(UAudioEvent::Type eventType,
			int deviceId,
			int fs,
			int frameLength,
			int channels,
			int bytesPerSample,
			bool overlap,
			int id) :
	_eventType(eventType),
	_recorder(0),
	_frameRate(0.0f),
	_simulateFreq(false),
	_overlap(overlap),
	_out(0),
	_p(0),
	_id(id)
{
	UASSERT(eventType == UAudioEvent::kTypeFrame || eventType == UAudioEvent::kTypeFrameFreq || eventType == UAudioEvent::kTypeFrameFreqSqrdMagn);
	UASSERT(deviceId >= 0);
	UASSERT(frameLength > 0);

	_recorder = new UAudioCaptureMic(deviceId, fs, frameLength, bytesPerSample, channels);

	// init FFTW stuff
	if(_recorder->frameLength())
	{
		int N = _overlap?frameLength*2:frameLength;
		_in.resize(N);
		_out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * N);
		_p = fftwf_plan_dft_r2c_1d(N, _in.data(), _out, 0);
		_window = uHamming(N);
	}
}

UAudioCaptureFFT::UAudioCaptureFFT(UAudioEvent::Type eventType,
			const std::string & path,
			float frameRate,
			bool simulateFrameRate,
			bool playWhileRecording,
			bool overlap,
			int id) :
	_eventType(eventType),
	_recorder(0),
	_frameRate(frameRate),
	_simulateFreq(simulateFrameRate),
	_overlap(overlap),
	_out(0),
	_p(0),
	_id(id)
{
	UASSERT(eventType == UAudioEvent::kTypeFrame || eventType == UAudioEvent::kTypeFrameFreq || eventType == UAudioEvent::kTypeFrameFreqSqrdMagn);

	if(playWhileRecording)
	{
		simulateFrameRate = false;
	}
	_recorder = new UAudioCaptureFile(path, playWhileRecording);

	//FFTW stuff not initialized because it requires UAudioRecorderFile to be initialized
}

UAudioCaptureFFT::~UAudioCaptureFFT()
{
	UDEBUG("");
	join(true);
	if(_recorder)
	{
		delete _recorder;
	}

	if(_out)
	{
		fftwf_destroy_plan((fftwf_plan)_p);
		fftwf_free(_out);
		_out = 0;
	}
}

bool UAudioCaptureFFT::init()
{
	_lastFrame.clear();

	if(!_recorder->init())
	{
		UERROR("Recorder initialization failed!");
		return false;
	}

	if(_frameRate>0.0f)
	{
		// If it is a UAudioRecorderFile, fs should be set here
		int frameLength = float(_recorder->fs())/_frameRate;
		UDEBUG("frameLength set to %d (fs=%d, hz=%f)", frameLength, _recorder->fs(), _frameRate);
		_recorder->setFrameLength(frameLength);
	}

	// init FFTW stuff
	if(_out)
	{
		fftwf_destroy_plan((fftwf_plan)_p);
		_p = 0;
		fftwf_free(_out);
		_out = 0;
		_in.clear();
		_window.clear();
	}
	int N = _overlap?_recorder->frameLength()*2:_recorder->frameLength();
	_in.resize(N);
	_out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * N);
	_p = fftwf_plan_dft_r2c_1d(N, _in.data(), _out, 0);
	_window = uHamming(N);

	return true;
}

void UAudioCaptureFFT::setPositionMs(unsigned int pos)
{
	if(_recorder && dynamic_cast<UAudioCaptureFile*>(_recorder))
	{
		UAudioCaptureFile * fileRecorder = (UAudioCaptureFile*)_recorder;
		fileRecorder->setPositionMs(pos);
	}
}

void UAudioCaptureFFT::stop()
{
	if(this->isRunning())
	{
		this->kill();
	}
	else if(_recorder && _recorder->isRunning())
	{
		_recorder->join(true);
	}
}

void UAudioCaptureFFT::startRecorder()
{
	if(_recorder)
	{
		_recorder->start();
		_timer.start();
	}
}

void UAudioCaptureFFT::mainLoopBegin()
{
	this->startRecorder();
}

void UAudioCaptureFFT::mainLoop()
{
	if(!_recorder)
	{
		UERROR("Recorder not initialized");
		this->kill();
		return;
	}

	if(this->isRunning())
	{
		bool noMoreFrames = true;
		if(_eventType == UAudioEvent::kTypeFrame)
		{
			std::vector<std::vector<char> > data = this->getFrame();
			if(data.size())
			{
				noMoreFrames = false;
				UEventsManager::post(new UAudioEvent(data, _recorder->bytesPerSample(), _recorder->fs(), _recorder->channels(), _id));
			}
		}
		else if(_eventType == UAudioEvent::kTypeFrameFreq)
		{
			std::vector<std::vector<float> > freq;
			std::vector<std::vector<char> > data = this->getFrame(freq, false);
			if(data.size() && freq.size())
			{
				noMoreFrames = false;
				UEventsManager::post(new UAudioEvent(UAudioEvent::kTypeFrameFreq, freq, _recorder->fs(), _recorder->channels(), _id));
			}
		}
		else if(_eventType == UAudioEvent::kTypeFrameFreqSqrdMagn)
		{
			std::vector<std::vector<float> > freq;
			std::vector<std::vector<char> > data = this->getFrame(freq, true);
			if(data.size() && freq.size())
			{
				noMoreFrames = false;
				UEventsManager::post(new UAudioEvent(UAudioEvent::kTypeFrameFreqSqrdMagn, freq, _recorder->fs(), _recorder->channels(), _id));
			}
		}
		else
		{
			UFATAL("Not supposed to be here...");
		}

		if(noMoreFrames)
		{
			if(this->isRunning())
			{
				UEventsManager::post(new UAudioEvent(_id));
			}
			this->kill();
		}
	}
}

void UAudioCaptureFFT::mainLoopKill()
{
	if(_recorder)
	{
		_recorder->join(true);
	}
}

std::vector<std::vector<char> > UAudioCaptureFFT::getFrame()
{
	std::vector<std::vector<char> > data;
	std::vector<char> frame;
	if(!_recorder)
	{
		UERROR("Micro is not initialized...");
		return data;
	}
	int frameLength = _recorder->frameLength();
	int fs = _recorder->fs();
	int channels = _recorder->channels();
	int bytesPerSample = _recorder->bytesPerSample();

	if(_simulateFreq && fs)
	{
		int sleepTime = ((double(frameLength)/double(fs) - _timer.getElapsedTime())  * 1000.0) + 0.5;
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}
		// Add precision at the cost of a small overhead
		while(_timer.getElapsedTime() < double(frameLength)/double(fs)-0.000001)
		{
			//
		}
		//double slept = _timer.getElapsedTime();
		_timer.start();
		//UDEBUG("slept=%fs vs target=%fs", slept, double(frameLength)/double(fs));
	}

	if(_recorder->getNextFrame(frame, true) && int(frame.size()) == frameLength * channels * bytesPerSample)
	{
		bool ok = true;
		if(_overlap && _lastFrame.size() == 0)
		{
			_lastFrame = frame;
			if(!_recorder->getNextFrame(frame, true) && int(frame.size()) == frameLength * channels * bytesPerSample)
			{
				UDEBUG("No more frames...");
				ok = false;
			}
		}
		if(ok)
		{
			if(_overlap)
			{
				std::vector<char> tmp = frame;
				frame.resize(_lastFrame.size() + tmp.size());
				memcpy(frame.data(), _lastFrame.data(), _lastFrame.size());
				memcpy(frame.data()+_lastFrame.size(), tmp.data(), tmp.size());
				_lastFrame = tmp;
				frameLength*=2;
			}
			// Split channels in rows
			//UDEBUG("frameLength=%d, bytesPerSample=%d, channels=%d", frameLength , bytesPerSample, channels);
			data = std::vector<std::vector<char> >(channels, std::vector<char>(frameLength*bytesPerSample));
			for(unsigned int i = 0; i<frame.size(); i+=channels*bytesPerSample)
			{
				for(unsigned int j=0; j<(unsigned int)channels; ++j)
				{
					memcpy(data[j].data() + i/channels, frame.data() + i + j*bytesPerSample, bytesPerSample);
				}
			}
		}
	}
	else
	{
		UDEBUG("No more frames...");
	}
	return data;
}

std::vector<std::vector<char> > UAudioCaptureFFT::getFrame(std::vector<std::vector<float> > & frameFreq, bool sqrdMagn)
{
	std::vector<std::vector<char> > frame = this->getFrame();
	frameFreq = this->computeFFT(frame, sqrdMagn);
	return frame;
}

std::vector<std::vector<float> > UAudioCaptureFFT::computeFFT(const std::vector<std::vector<char> > & frame, bool sqrdMagn)
{
	std::vector<std::vector<float> > frameFreq;
	if(!frame.size())
	{
		return frameFreq;
	}

	if(!_p || _window.empty() || _in.empty() || !_out)
	{
		UWARN("Micro not initialized!");
		return frameFreq;
	}
	else if((int)_window.size()!=int(frame[0].size())/_recorder->bytesPerSample())
	{
		UWARN("Frame (%d) and initialized window (%d) are not the same size!", frame[0].size(), _window.size());
		return frameFreq;
	}
	if(!frame.empty())
	{
		std::vector<std::vector<float> > timeSample(frame.size(), std::vector<float>(frame[0].size()/_recorder->bytesPerSample()));
		// for each channel
		for(unsigned int j=0; j<frame.size(); ++j)
		{
			for(unsigned int i=0; i<frame[j].size(); i+=_recorder->bytesPerSample())
			{
				if(_recorder->bytesPerSample() == 1)
				{
					timeSample.at(j).at(i/_recorder->bytesPerSample()) = _window[i/_recorder->bytesPerSample()] * float(frame.at(j).at(i)) / float(1<<7); // between 0 and 1
				}
				else if(_recorder->bytesPerSample() == 2)
				{
					timeSample.at(j).at(i/_recorder->bytesPerSample()) = _window[i/_recorder->bytesPerSample()] * float(*(short*)&frame.at(j).at(i)) / float(1<<15); // between 0 and 1
				}
				else if(_recorder->bytesPerSample() == 4)
				{
					timeSample.at(j).at(i/_recorder->bytesPerSample()) = _window[i/_recorder->bytesPerSample()] * float(*(int*)&frame.at(j).at(i)) / float(1<<31); // between 0 and 1
				}
			}
		}

		int size = timeSample[0].size()/2+1;
		if(sqrdMagn)
		{
			frameFreq.resize(timeSample.size(), std::vector<float>(size));
		}
		else
		{
			frameFreq.resize(timeSample.size(), std::vector<float>(size*2)); //[re, im, re, im, ...]
		}

		// for each channel
		for(unsigned int j=0; j<timeSample.size(); ++j)
		{
			std::vector<float> & row = timeSample.at(j);
			std::vector<float> & rowFreq = frameFreq.at(j);
			memcpy(_in.data(), row.data(), row.size()*sizeof(float));
			fftwf_execute((fftwf_plan)_p); /* repeat as needed */

			float re;
			float im;
			for(int i=0; i<size; ++i)
			{
				re = float(_out[i][0]);
				im = float(_out[i][1]);
				if(sqrdMagn)
				{
					rowFreq.at(i) = re*re+im*im; // squared magnitude
				}
				else
				{
					rowFreq.at(i*2) = re;
					rowFreq.at(i*2+1) = im;
				}
			}
		}
	}
	return frameFreq;
}

int UAudioCaptureFFT::fs()
{
	int fs = 0;
	if(_recorder)
	{
		fs = _recorder->fs();
	}
	return fs;
}

int UAudioCaptureFFT::bytesPerSample()
{
	int bytes = 0;
	if(_recorder)
	{
		bytes = _recorder->bytesPerSample();
	}
	return bytes;
}

int UAudioCaptureFFT::channels()
{
	int channels = 0;
	if(_recorder)
	{
		channels = _recorder->channels();
	}
	return channels;
}

int UAudioCaptureFFT::samples()
{
	int n = 0;
	if(_recorder)
	{
		n = _recorder->samples();
	}
	return n;
}
