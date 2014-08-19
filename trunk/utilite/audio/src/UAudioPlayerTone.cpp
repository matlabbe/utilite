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

#include "utilite/UAudioPlayerTone.h"
#include "UAudioSystem.h"
#include <fmodex/fmod_errors.h>
#include <utilite/UTimer.h>
#include <utilite/ULogger.h>

UAudioPlayerTone::UAudioPlayerTone(float freq, 
                           float vol, 
                           unsigned int toneMs, 
                           unsigned int tickMs) :
    UAudioPlayer(""),
    _freq(freq),
    _dsp(0),
    _toneMs(toneMs),
    _tickMs(tickMs),
    _vol(vol)
{  
    UDEBUG("");
}

UAudioPlayerTone::UAudioPlayerTone(const char* filePath, 
                           float vol,  
                           unsigned int tickMs) :
    UAudioPlayer(filePath),
    _freq(0),
    _dsp(0),
    _toneMs(0),
    _tickMs(tickMs),
    _vol(vol)
{
    UDEBUG("");
}

UAudioPlayerTone::~UAudioPlayerTone()
{
    UDEBUG("");
    this->stop();
}

int UAudioPlayerTone::init()
{
    if(_freq == 0)
    {
        return UAudioPlayer::init();
    }

    FMOD_RESULT result;
    result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    if(_channel)
    {
        result = FMOD_Channel_Stop(_channel);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
    }

    if(_dsp)
    {
        result = FMOD_DSP_Release(_dsp);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    }

    //Create an oscillator DSP units for the tone.
    result = UAudioSystem::createDSPByType(FMOD_DSP_TYPE_OSCILLATOR, &_dsp);     UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    // 440.0f = musical note 'A'
    result = FMOD_DSP_SetParameter(_dsp, FMOD_DSP_OSCILLATOR_RATE, _freq);       UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    
    // Set paused
    result = UAudioSystem::playDSP(FMOD_CHANNEL_FREE, _dsp, true, &_channel);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    result = FMOD_Channel_SetVolume(_channel, _vol);                                 UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    // 0 = sinus
    result = FMOD_DSP_SetParameter(_dsp, FMOD_DSP_OSCILLATOR_TYPE, 0);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    FMOD_Channel_SetMute(_channel, true);
    FMOD_Channel_SetPaused(_channel, false);

    return 0;
}

void UAudioPlayerTone::play()
{
    this->init();
    this->start();
}

void UAudioPlayerTone::pause()
{
    
}

void UAudioPlayerTone::stop()
{
    this->join(true);
}

unsigned int UAudioPlayerTone::positionMs()
{
    return 0;
}

void UAudioPlayerTone::setPositionMs(unsigned int pos)
{
}

void UAudioPlayerTone::mainLoopBegin()
{
}

void UAudioPlayerTone::mainLoop()
{
    FMOD_RESULT result;

    if(_dsp)
    {
        result = FMOD_Channel_SetMute(_channel, false);       UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        uSleep(_toneMs);
        if(!this->isKilled())
        {
			result = FMOD_Channel_SetMute(_channel, true);       UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
			uSleep(_tickMs);
        }
    }
    else
    {
        result = UAudioSystem::playSound(FMOD_CHANNEL_REUSE, _sound, false, &_channel);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        uSleep(_tickMs);
    }
    UAudioSystem::update();
}

void UAudioPlayerTone::mainLoopEnd()
{
	// This will stop the channel and the sound
	UAudioPlayer::stop();

	FMOD_RESULT result;
	if(_dsp)
	{
		result = FMOD_DSP_Release(_dsp);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		_dsp = 0;
	}
}
