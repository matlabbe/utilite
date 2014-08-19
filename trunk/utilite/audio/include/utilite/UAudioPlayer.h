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
