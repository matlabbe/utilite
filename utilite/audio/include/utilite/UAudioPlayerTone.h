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

#ifndef UTONEAUDIOPLAYER_H
#define UTONEAUDIOPLAYER_H

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include "utilite/UAudioPlayer.h"
#include <utilite/UThreadNode.h>
#include <string>

class FMOD_DSP;

/**
 * This player is used to generate a 
 * tone (with variable frequency and length). It will 
 * use the DSP system of FMOD to generate a sound. It can be 
 * used with a file to repeat it after each a defined tick in ms.
 */
class UTILITEAUDIO_EXP UAudioPlayerTone : public UAudioPlayer, public UThread
{
public:
    UAudioPlayerTone(float freq = 440.0f, float vol = 0.5f,  unsigned int toneMs = 200, unsigned int tickMs = 200);
    UAudioPlayerTone(const char* filePath, float vol = 0.5f,  unsigned int tickMs = 200);

    virtual ~UAudioPlayerTone();

    virtual void play();
    virtual void pause();
    virtual void stop();
    virtual unsigned int positionMs();

    virtual void setPositionMs(unsigned int pos);

    //setters
    void setFreq(float freq) {_freq = freq;}
    void setVol(float vol) {_vol = vol;}
    void setToneMs(unsigned int toneMs) {_toneMs = toneMs;}
    void setSilenceMs(unsigned int tickMs) {_tickMs = tickMs;}

    //getters
    float getFreq() {return _freq;}
    float getVol() {return _vol;}
    unsigned int getToneMs() {return _toneMs;}
    unsigned int getTickMs() {return _tickMs;}
    virtual int init();

protected:

private:

    virtual void mainLoopBegin();
    virtual void mainLoop();
    virtual void mainLoopEnd();

private:
    float _freq;
    FMOD_DSP * _dsp;

    unsigned int _toneMs;
    unsigned int _tickMs;
    float _vol;

};

#endif
