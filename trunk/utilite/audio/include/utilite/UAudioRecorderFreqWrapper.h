/*
 * Micro.h
 *
 *  Created on: Mar 5, 2012
 *      Author: MatLab
 */

#ifndef MICRO_H_
#define MICRO_H_

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include "utilite/UThreadNode.h"
#include "utilite/UEvent.h"
#include "utilite/UTimer.h"
#include <string>
#include <vector>

class UAudioRecorder;

class UTILITE_EXP UAudioEvent :
	public UEvent
{
public:
	enum Type {
		kTypeFrame,
		kTypeFrameFreq,
		kTypeFrameFreqSqrdMagn,
		kTypeNoMoreFrames
	};

public:
	// kTypeNoMoreFrames constructor
	UAudioEvent(int microId = 0);
	// kTypeFrame constructor
	UAudioEvent(const std::vector<std::vector<char> > & frame,
			int sampleSize,
			int fs,
			int channels,
			int microId = 0);
	// kTypeFrameFreq and kTypeFrameFreqSqrdMagn constructors
	UAudioEvent(Type frameType,
			const std::vector<std::vector<float> > & frameFreq,
			int fs,
			int channels,
			int microId = 0);

	int type() const {return this->getCode();}
	const std::vector<std::vector<char> > & frame() const {return _frame;}
	const std::vector<std::vector<float> > & frameFreq() const {return _frameFreq;}
	int sampleSize() const {return _sampleSize;}
	int microId() const {return _microId;}
	int fs() const {return _fs;}
	virtual ~UAudioEvent() {}
	virtual std::string getClassName() const {return std::string("UAudioEvent");}

private:
	std::vector<std::vector<char> > _frame;
	std::vector<std::vector<float> > _frameFreq;
	int _sampleSize; // bytes
	int _fs; //sampling rate
	int _microId;
};

class UTILITE_EXP UAudioRecorderFreqWrapper : public UThreadNode
{
	typedef float fftwf_complex[2];
public:
	UAudioRecorderFreqWrapper(UAudioEvent::Type eventType,
			int deviceId,
			int fs,
			int frameLength,
			int channels,
			int bytesPerSample,
			bool overlap = false,
			int id = 0);
	UAudioRecorderFreqWrapper(UAudioEvent::Type eventType,
			const std::string & path,
			float frameRate,
			bool simulateFrameRate = true,
			bool playWhileRecording = false,
			bool overlap = false,
			int id = 0);
	virtual ~UAudioRecorderFreqWrapper();

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
	UAudioRecorder* _recorder;
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
