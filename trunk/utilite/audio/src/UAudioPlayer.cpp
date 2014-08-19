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

#include "utilite/UAudioPlayer.h"
#include "UAudioSystem.h"
#include <fmodex/fmod_errors.h>
#include <utilite/ULogger.h>
#include <utilite/UFile.h>

#include <string>

UAudioPlayer::UAudioPlayer() :
    _sound(0),
    _channel(0),
    _fileName("")
{  
	UAudioSystem::acquire();
    UDEBUG("Player::Player()");
}

UAudioPlayer::UAudioPlayer(const std::string & fileName) :
    _sound(0),
    _channel(0),
    _fileName(fileName)
{  
	UAudioSystem::acquire();
    UDEBUG("");
}

UAudioPlayer::~UAudioPlayer()
{
    UDEBUG("");
    this->stop();
    UAudioSystem::release();
}

int UAudioPlayer::init()
{
    if(!UFile::exists(_fileName))
    {
        return 1;
    }

    FMOD_RESULT result;
    if(_channel)
    {
        result = FMOD_Channel_Stop(_channel);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
    }

    if(_sound)
    {
        result = FMOD_Sound_Release(_sound);     UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    }
    
    result = UAudioSystem::createStream(_fileName.c_str(), FMOD_SOFTWARE, 0, &_sound);     UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    result = UAudioSystem::playSound(FMOD_CHANNEL_REUSE, _sound, true, &_channel);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    //result = FMOD_Sound_(_sound->setMode(FMOD_LOOP_OFF);    /* drumloop.wav has embedded loop points which automatically makes looping turn on, */
    //ERRCHECK(result);                           /* so turn it off here.  We could have also just put FMOD_LOOP_OFF in the above CreateSound call. */
    return 0;
}

void UAudioPlayer::play()
{
    FMOD_RESULT result;
    result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    if(_sound == 0)
    {
        if(this->init() != 0)
        {
            UERROR("init() failed");
            return;
        }
        
    }

    // Paused ?
    FMOD_BOOL isPlaying;
    result = FMOD_Channel_IsPlaying(_channel, &isPlaying);
    if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
    {
        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    }
    if(!isPlaying)
    {
        result = UAudioSystem::playSound(FMOD_CHANNEL_REUSE, _sound, false, &_channel);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    }
    else
    {
        // Paused ?
    	FMOD_BOOL paused;
        result = FMOD_Channel_GetPaused(_channel, &paused);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }

        if(!paused)
        {
            // Return to start
            result = FMOD_Channel_SetPosition(_channel, 0, FMOD_TIMEUNIT_MS);
            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
            {
                UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            }
        }
        else
        {
            // Unpaused
            result = FMOD_Channel_SetPaused(_channel, false);
            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
            {
                UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            }
        }
    }
}

void UAudioPlayer::pause()
{
    FMOD_RESULT result;
    result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    if(_channel)
    {
    	FMOD_BOOL paused;

        // Get paused flag
        result = FMOD_Channel_GetPaused(_channel, &paused);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }

        // Set paused flag
        result = FMOD_Channel_SetPaused(_channel, !paused);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
    }
}

void UAudioPlayer::stop()
{
    UDEBUG("");
    FMOD_RESULT result;
    result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    if(_channel)
    {
        result = FMOD_Channel_Stop(_channel);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
        _channel=0;
    }

    if(_sound)
    {
        result = FMOD_Sound_Release(_sound);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
        _sound=0;
    }
}

unsigned int UAudioPlayer::positionMs()
{
    UDEBUG("Player::positionMs()");
    FMOD_RESULT result;
    result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    //Pour avoir la position
    unsigned int ms = 0;
    if(_channel)
    {
        result = FMOD_Channel_GetPosition(_channel, &ms, FMOD_TIMEUNIT_MS);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
    }
    return ms;
}

void UAudioPlayer::setPositionMs(unsigned int pos)
{
    UDEBUG("(%d)", pos);
    FMOD_RESULT result;
    result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    //Pour modifier la position
    if(_channel)
    {
        // Get paused flag
    	FMOD_BOOL paused;
        result = FMOD_Channel_GetPaused(_channel, &paused);                    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        if(paused)
        {
            result = FMOD_Channel_SetPosition(_channel, pos, FMOD_TIMEUNIT_MS);    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
        else
        {
            result = FMOD_Channel_SetPaused(_channel, true);                        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            result = FMOD_Channel_SetPosition(_channel, pos, FMOD_TIMEUNIT_MS);    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            result = FMOD_Channel_SetPaused(_channel, false);                    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
    }
}

unsigned int UAudioPlayer::getSoundLengthMs()
{
    UDEBUG("");
    FMOD_RESULT result;
    unsigned int length = 0;
    if(_sound)
    {
        result = FMOD_Sound_GetLength(_sound, &length, FMOD_TIMEUNIT_MS);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    } 
    return length;
}


bool UAudioPlayer::isPlaying()
{
    UDEBUG("");
    FMOD_RESULT result;
    FMOD_BOOL isPlaying = false;
    if(_channel)
    {
        result = FMOD_Channel_IsPlaying(_channel, &isPlaying);    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    }
    return (bool)isPlaying;
}
