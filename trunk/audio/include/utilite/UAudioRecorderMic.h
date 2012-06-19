/**
 * @file UAudioRecorderMic.h
 *
 * @author Unknown, Modified by Mathieu Labbe
 */

#ifndef UMICAUDIORECORDER_H
#define UMICAUDIORECORDER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include "utilite/UAudioRecorder.h"
#include <string>
#include <vector>

class FMOD_SOUND;

/**
 * This recorder gets frames from the microphone. It is 
 * configured with the driver number of the microphone 
 * and the frequency. You can retrieve the list of available
 * drivers on the computer with the method getRecordDrivers(). 
 * It can be also created to record in a file while getting frames 
 * from the microphone stream. It records by default in wav but 
 * if the file name contains ".mp3", it will automatically
 * encode when the process is finished. The recording is 
 * finished when the maximum size set is reached.
 *
 * \n Example recording to a file and processing the frame: 
 * @code
 *    UAudioRecorder* recorder = new PlayRecorder("fileName.wav", 100);
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
 * @see UAudioRecorderMic::getRecordDrivers
 *
 */
class UTILITE_EXP UAudioRecorderMic : public UAudioRecorder
{
public:
	/**
	 * Record from mic.
	 */
    UAudioRecorderMic(int driver = 0,
    			int fs = 44100,
    			int frameLength = 1024,
    			int bytesPerSample = 2,
    			int channels = 1);

    /**
     * Record from mic while saving frames to a file.
     */
    UAudioRecorderMic(const std::string &fileName,
                int maxFileSizeMb,
                int driver = 0,
                int fs = 44100,
				int frameLength = 1024,
				int bytesPerSample = 2,
				int channels = 1);

    virtual ~UAudioRecorderMic();

    virtual bool init();
    virtual void close();
    const std::string &getFileName() const { return _fileName; }
    int getDriverId() const { return _driver; }

public:
    static std::vector<std::string> getRecordDrivers();

protected:
    virtual void mainLoopBegin();
    virtual void mainLoop();
    virtual void mainLoopEnd();

private:    
    int _driver;

	std::string _fileName;
	int _maxFileSize;
	bool _encodeToMp3;
	FILE *_fp;

	FMOD_SOUND * _sound;

	unsigned int _dataLength;
	unsigned int _soundLength;
	unsigned int _lastRecordPos;
};

#endif
