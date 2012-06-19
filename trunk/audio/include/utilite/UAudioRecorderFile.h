/**
 * @file UAudioRecorderFile.h
 *
 * @author Modified by Mathieu Labbe
 */

#ifndef UFILEAUDIORECORDER_H
#define UFILEAUDIORECORDER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include "utilite/UAudioRecorder.h"
#include <string>

class FMOD_SOUND;
class FMOD_CHANNEL;

/**
 * This recorder loads frames from an audio file in 
 * the samples list of the UAudioRecorder. Almost all 
 * audio file types are supported (see Fmod audio 
 * types supported).
 *
 * \n Example : 
 * @code
 *    UAudioRecorder* recorder = new UAudioRecorderFile("fileName.wav");
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
class UTILITE_EXP UAudioRecorderFile : public UAudioRecorder
{
public:
    UAudioRecorderFile(const std::string &fileName, 
                 bool playSoundWhileRecording = false,
				 int frameLength = 1024);

    virtual ~UAudioRecorderFile();

    const std::string &getFileName() const {return _fileName;}

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
