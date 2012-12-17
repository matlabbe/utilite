/*
 * UAudioEvent.cpp
 *
 *  Created on: 2012-12-07
 *      Author: mathieu
 */

#include "utilite/UAudioEvent.h"
#include "utilite/ULogger.h"

// kTypeNoMoreFrames constructor
UAudioEvent::UAudioEvent(int microId) :
	UEvent(kTypeNoMoreFrames),
	_sampleSize(0),
	_fs(0),
	_microId(microId)
{
}
// kTypeFrame constructor
UAudioEvent::UAudioEvent(const std::vector<std::vector<char> > & frame,
		int sampleSize,
		int fs,
		int channels,
		int microId) :
	UEvent(kTypeFrame),
	_frame(frame),
	_sampleSize(sampleSize),
	_fs(0),
	_microId(microId)
{
}
// kTypeFrameFreq and kTypeFrameFreqSqrdMagn constructors
UAudioEvent::UAudioEvent(Type frameType,
		const std::vector<std::vector<float> > & frameFreq,
		int fs,
		int channels,
		int microId) :
	UEvent(frameType),
	_frameFreq(frameFreq),
	_sampleSize(sizeof(float)),
	_fs(0),
	_microId(microId)
{
	UASSERT(frameType == kTypeFrameFreqSqrdMagn || frameType == kTypeFrameFreq);
}
