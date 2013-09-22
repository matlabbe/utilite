/*
 * Micro.h
 *
 *  Created on: Mar 5, 2012
 *      Author: MatLab
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
