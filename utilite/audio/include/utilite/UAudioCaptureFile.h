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

#ifndef UFILEAUDIORECORDER_H
#define UFILEAUDIORECORDER_H

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include "utilite/UAudioCapture.h"
#include <string>

class FMOD_SOUND;
class FMOD_CHANNEL;

/**
 * This recorder loads frames from an audio file in 
 * the samples list of the UAudioCapture. Almost all
 * audio file types are supported (see Fmod audio 
 * types supported).
 *
 * \n Example : 
 * @code
 *    UAudioCapture* recorder = new UAudioCaptureFile("fileName.wav");
 *    Frame frame;
 *    unsigned int frameId = 0;
 *    recorder->start();
 *    while(recorder->getNextFrame(frame, frameId))
 *    { 
 *      ...process the frame...
 *      ... break when a condition is reached...
 *    }
 *    recorder->stop();
 *      delete recorder;
 * @endcode
 *
 *
 */
class UTILITEAUDIO_EXP UAudioCaptureFile : public UAudioCapture
{
public:
    UAudioCaptureFile(const std::string &fileName, 
                 bool playSoundWhileRecording = false,
				 int frameLength = 1024);

    virtual ~UAudioCaptureFile();

    const std::string &getFileName() const {return _fileName;}
    void setPositionMs(unsigned int pos);

    virtual bool init();
    virtual void close();

protected:
    virtual void mainLoopBegin();
    virtual void mainLoop();
    virtual void mainLoopEnd();

private:
    std::string _fileName;
    FMOD_SOUND * _sound;
    unsigned int _soundLength;

    bool _playSoundWhileRecording;
    unsigned int _dataLength;
	unsigned int _lastRecordPos;
	FMOD_CHANNEL * _channel;
};

#endif
