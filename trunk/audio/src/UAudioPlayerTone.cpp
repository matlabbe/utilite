#include "utilite/UAudioPlayerTone.h"
#include "System.h"
#include <fmod_errors.h>
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
    result = System::update();         System::ERRCHECK(result);

    if(_channel)
    {
        result = _channel->stop();
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }
    }

    if(_dsp)
    {
        result = _dsp->release();   System::ERRCHECK(result);
    }

    //Create an oscillator DSP units for the tone.
    result = System::createDSPByType(FMOD_DSP_TYPE_OSCILLATOR, &_dsp);     System::ERRCHECK(result);

    // 440.0f = musical note 'A'
    result = _dsp->setParameter(FMOD_DSP_OSCILLATOR_RATE, _freq);       System::ERRCHECK(result);
    
    // Set paused
    result = System::playDSP(FMOD_CHANNEL_FREE, _dsp, true, &_channel);   System::ERRCHECK(result);
    result = _channel->setVolume(_vol);                                 System::ERRCHECK(result);
    // 0 = sinus
    result = _dsp->setParameter(FMOD_DSP_OSCILLATOR_TYPE, 0);   System::ERRCHECK(result);

    _channel->setMute(true);
    _channel->setPaused(false);

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
        result = _channel->setMute(false);       System::ERRCHECK(result);
        uSleep(_toneMs);
        result = _channel->setMute(true);       System::ERRCHECK(result);
        uSleep(_tickMs);
    }
    else
    {
        result = System::playSound(FMOD_CHANNEL_REUSE, _sound, false, &_channel);   System::ERRCHECK(result);
        uSleep(_tickMs);
    }
}

void UAudioPlayerTone::mainLoopKill()
{
    // This will stop the channel and the sound
    UAudioPlayer::stop();

    FMOD_RESULT result;
    if(_dsp)
    {
        result = _dsp->release();        System::ERRCHECK(result);
        _dsp = 0;
    }
}
