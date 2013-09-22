/*
 * UAudioEvent.h
 *
 *  Created on: 2012-12-07
 *      Author: mathieu
 */

#ifndef UAUDIOEVENT_H_
#define UAUDIOEVENT_H_

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include <utilite/UEvent.h>
#include <vector>

/**
 * An audio event.
 */
class UTILITEAUDIO_EXP UAudioEvent :
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

#endif /* UAUDIOEVENT_H_ */
