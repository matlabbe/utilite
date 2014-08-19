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

#ifndef MICRO_H_
#define MICRO_H_

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include "utilite/UThreadNode.h"
#include "utilite/UAudioEvent.h"
#include "utilite/UTimer.h"
#include <string>
#include <vector>

class UAudioCapture;

/**
 * Wrapper of UAudioCapture to get samples in frequency domain.
 * Info: FFTW used.
 */
class UTILITEAUDIO_EXP UAudioCaptureFFT : public UThread
{
	typedef float fftwf_complex[2];
public:
	UAudioCaptureFFT(UAudioEvent::Type eventType,
			int deviceId,
			int fs,
			int frameLength,
			int channels,
			int bytesPerSample,
			bool overlap = false,
			int id = 0);
	UAudioCaptureFFT(UAudioEvent::Type eventType,
			const std::string & path,
			float frameRate,
			bool simulateFrameRate = true,
			bool playWhileRecording = false,
			bool overlap = false,
			int id = 0);
	virtual ~UAudioCaptureFFT();

	bool init();
	void stop(); // same as kill() but handle the case where underlying recorder is running and not the micro.
	void startRecorder(); // must only be used if Micro::start() is not used

	void setPositionMs(unsigned int pos);

	std::vector<std::vector<char> > getFrame();
	std::vector<std::vector<char> > getFrame(std::vector<std::vector<float> > & frameFreq, bool sqrdMagn = false);
	std::vector<std::vector<float> > computeFFT(const std::vector<std::vector<char> > & frame, bool sqrdMagn = false);

	int fs();
	int bytesPerSample();
	int channels();
	int nfft();
	int samples();
	float frameRate() const {return _frameRate;}

protected:
	virtual void mainLoopBegin();
	virtual void mainLoop();
	virtual void mainLoopKill();

private:
	UAudioEvent::Type _eventType;
	UAudioCapture* _recorder;
	float _frameRate;
	bool _simulateFreq;
	bool _overlap;
	UTimer _timer;
	std::vector<float> _window;
	std::vector<float> _in;
	fftwf_complex * _out; // fftwf_complex
	void * _p; // fftwf_plan
	int _id;
	std::vector<char> _lastFrame;
};

#endif /* MICRO_H_ */
