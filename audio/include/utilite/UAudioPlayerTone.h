/**
 * @file UAudioPlayerTone.h
 *
 * @author Mathieu Labbe
 */

#ifndef UTONEAUDIOPLAYER_H
#define UTONEAUDIOPLAYER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

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
class UTILITE_EXP UAudioPlayerTone : public UAudioPlayer, public UThreadNode
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
