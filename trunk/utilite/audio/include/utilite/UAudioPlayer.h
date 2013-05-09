/**
 * @file UAudioPlayer.h
 *
 * @author Mathieu Labbe
 */

#ifndef UAUDIOPLAYER_H
#define UAUDIOPLAYER_H

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include <string>

class FMOD_SOUND;
class FMOD_CHANNEL;


/**
 * A player is used to play a sound. It works 
 * like other players like Winamp, Windows Media UAudioPlayer.
 * Almost all sound formats are supported (wav, mp3, 
 * mid, ...), see FMOD documentation.
 */
class UTILITEAUDIO_EXP UAudioPlayer
{
public:
    UAudioPlayer();
    UAudioPlayer(const std::string & fileName);

    void setFileName(const std::string & fileName) {_fileName = fileName;}
    const std::string & getFileName() {return _fileName;}

    virtual ~UAudioPlayer();

    virtual void play();
    virtual void pause();
    virtual void stop();
    virtual unsigned int positionMs();
    virtual unsigned int getSoundLengthMs();

    virtual bool isPlaying();

    virtual void setPositionMs(unsigned int pos);

    virtual int init();

protected:

protected:
    FMOD_SOUND * _sound;
    FMOD_CHANNEL * _channel;;
    std::string _fileName;
};

#endif
