#include "utilite/UAudioPlayer.h"
#include "System.h"
#include <fmod_errors.h>
#include <utilite/ULogger.h>
#include <utilite/UFile.h>

#include <string>

UAudioPlayer::UAudioPlayer() :
    _sound(0),
    _channel(0),
    _fileName("")
{  
    UDEBUG("Player::Player()");
}

UAudioPlayer::UAudioPlayer(const std::string & fileName) :
    _sound(0),
    _channel(0),
    _fileName(fileName)
{  
    UDEBUG("");
}

UAudioPlayer::~UAudioPlayer()
{
    UDEBUG("");
    this->stop();
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
        result = _channel->stop();
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }
    }

    if(_sound)
    {
        result = _sound->release();     System::ERRCHECK(result);
    }
    
    result = System::createStream(_fileName.c_str(), FMOD_SOFTWARE, 0, &_sound);     System::ERRCHECK(result);

    result = System::playSound(FMOD_CHANNEL_REUSE, _sound, true, &_channel);   System::ERRCHECK(result);

    //result = FMOD_Sound_(_sound->setMode(FMOD_LOOP_OFF);    /* drumloop.wav has embedded loop points which automatically makes looping turn on, */
    //ERRCHECK(result);                           /* so turn it off here.  We could have also just put FMOD_LOOP_OFF in the above CreateSound call. */
    return 0;
}

void UAudioPlayer::play()
{
    FMOD_RESULT result;
    result = System::update();         System::ERRCHECK(result);

    if(_sound == 0)
    {
        if(this->init() != 0)
        {
            UERROR("init() failed");
            return;
        }
        
    }

    // Paused ?
    bool isPlaying;
    result = _channel->isPlaying(&isPlaying);
    if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
    {
        System::ERRCHECK(result);
    }
    if(!isPlaying)
    {
        result = System::playSound(FMOD_CHANNEL_REUSE, _sound, false, &_channel);   System::ERRCHECK(result);
    }
    else
    {
        // Paused ?
    	bool paused;
        result = _channel->getPaused(&paused);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }

        if(!paused)
        {
            // Return to start
            result = _channel->setPosition(0, FMOD_TIMEUNIT_MS);
            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
            {
                System::ERRCHECK(result);
            }
        }
        else
        {
            // Unpaused
            result = _channel->setPaused(false);
            if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
            {
                System::ERRCHECK(result);
            }
        }
    }
}

void UAudioPlayer::pause()
{
    FMOD_RESULT result;
    result = System::update();         System::ERRCHECK(result);

    if(_channel)
    {
    	bool paused;

        // Get paused flag
        result = _channel->getPaused(&paused);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }

        // Set paused flag
        result = _channel->setPaused(!paused);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }
    }
}

void UAudioPlayer::stop()
{
    UDEBUG("");
    FMOD_RESULT result;
    result = System::update();         System::ERRCHECK(result);

    if(_channel)
    {
        result = _channel->stop();
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }
        _channel=0;
    }

    if(_sound)
    {
        result = _sound->release();
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }
        _sound=0;
    }
}

unsigned int UAudioPlayer::positionMs()
{
    UDEBUG("Player::positionMs()");
    FMOD_RESULT result;
    result = System::update();         System::ERRCHECK(result);

    //Pour avoir la position
    unsigned int ms = 0;
    if(_channel)
    {
        result = _channel->getPosition(&ms, FMOD_TIMEUNIT_MS);
        if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
        {
            System::ERRCHECK(result);
        }
    }
    return ms;
}

void UAudioPlayer::setPositionMs(unsigned int pos)
{
    UDEBUG("(%d)", pos);
    FMOD_RESULT result;
    result = System::update();         System::ERRCHECK(result);

    //Pour modifier la position
    if(_channel)
    {
        // Get paused flag
    	bool paused;
        result = _channel->getPaused(&paused);                    System::ERRCHECK(result);
        if(paused)
        {
            result = _channel->setPosition(pos, FMOD_TIMEUNIT_MS);    System::ERRCHECK(result);
        }
        else
        {
            result = _channel->setPaused(true);                        System::ERRCHECK(result);
            result = _channel->setPosition(pos, FMOD_TIMEUNIT_MS);    System::ERRCHECK(result);
            result = _channel->setPaused(false);                    System::ERRCHECK(result);
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
        result = _sound->getLength(&length, FMOD_TIMEUNIT_MS);   System::ERRCHECK(result);
    } 
    return length;
}


bool UAudioPlayer::isPlaying()
{
    UDEBUG("");
    FMOD_RESULT result;
    bool isPlaying = false;
    if(_channel)
    {
        result = _channel->isPlaying(&isPlaying);    System::ERRCHECK(result);
    }
    return isPlaying;
}
