/**
 * @file UAudioCaptureFile.h
 *
 * @author Modified by Mathieu Labbe
 */

#ifndef UFILEAUDIORECORDER_H
#define UFILEAUDIORECORDER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

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
class UTILITE_EXP UAudioCaptureFile : public UAudioCapture
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
