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
