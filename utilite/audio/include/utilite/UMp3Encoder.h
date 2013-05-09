/**
 * @file UMp3Encoder.h
 *
 * @author Mathieu Labbe
 */

#ifndef UMP3ENCODER_H
#define UMP3ENCODER_H

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include <string>

/**
 * This class encodes a wav (PCM) to mp3. Inherited class 
 * can redefined the three virtual methods to have a 
 * feedback while the file is encoded.
 */
class UTILITEAUDIO_EXP UMp3Encoder
{
public:
    int encode(const std::string &fileIn, const std::string &fileOut, bool writeOver = true);

protected:
    virtual void encodeProcessBegin(long totalSamples) {}
    virtual void encodeProcess(long samplesWrited) {}
    virtual void encodeProcessEnd() {}

private:

};

#endif
