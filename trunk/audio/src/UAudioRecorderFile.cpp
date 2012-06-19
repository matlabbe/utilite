
#include "utilite/UAudioRecorderFile.h"
#include "UAudioSystem.h"
#include <utilite/UFile.h>
#include <utilite/ULogger.h>

UAudioRecorderFile::UAudioRecorderFile(const std::string &fileName, 
                           bool playSoundWhileRecording,
                           int frameLength) :
    UAudioRecorder(0, frameLength, 0, 0),
    _fileName(fileName),
    _sound(0),
    _soundLength(0),
    _playSoundWhileRecording(playSoundWhileRecording),
    _dataLength(0),
    _lastRecordPos(0),
    _channel(0)
{
    UDEBUG("");
}

UAudioRecorderFile::~UAudioRecorderFile()
{
    this->join(true);
    this->close();
}

void UAudioRecorderFile::close()
{
	FMOD_RESULT result;
	if(_channel)
	{
		result = FMOD_Channel_Stop(_channel);
		if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
		{
			UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		}
		_channel = 0;
	}

	if(_sound)
	{
		result = FMOD_Sound_Release(_sound);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		_sound = 0;
	}
}

bool UAudioRecorderFile::init()
{
	this->close();
	bool ok = UAudioRecorder::init();
	if(ok)
	{
		FMOD_RESULT result;
		int bitsPerSample = 0;
		int channels = 0;
		float fs = 0;

		_dataLength = 0;
		_soundLength = 0;
		_lastRecordPos = 0;

		if(UFile::exists(_fileName))
		{
			if(_playSoundWhileRecording)
			{
				result = UAudioSystem::createStream(_fileName.c_str(), FMOD_SOFTWARE, 0, &_sound);     UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
			}
			else
			{
				result = UAudioSystem::createStream(_fileName.c_str(), FMOD_OPENONLY | FMOD_CREATESAMPLE | FMOD_ACCURATETIME, 0, &_sound);    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
				FMOD_Sound_GetLength(_sound, &_soundLength, FMOD_TIMEUNIT_PCM); // in bytes
			}
			FMOD_Sound_GetFormat(_sound, 0, 0, &channels, &bitsPerSample);
			FMOD_Sound_GetDefaults(_sound, &fs, 0, 0, 0);
			UDEBUG("Using %d bits per sample, %d channels and %f sampling frequency", bitsPerSample, channels, fs);
			this->setBytesPerSample(bitsPerSample/8);
			this->setChannels(channels);
			this->setFs((int)fs);

			if(_playSoundWhileRecording)
			{
				result = UAudioSystem::playSound(FMOD_CHANNEL_FREE, _sound, true, &_channel);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
				UAudioSystem::update();
				FMOD_Sound_GetLength(_sound, &_soundLength, FMOD_TIMEUNIT_PCM); // in bytes
			}
		}
		else
		{
			UDEBUG("file %s doesn't exist", _fileName.c_str());
			return false;
		}

		if(!_playSoundWhileRecording)
		{
			unsigned int read = 0;
			char *buffer = 0;

			buffer = new char[_soundLength*bytesPerSample()*channels];

			result = FMOD_Sound_SeekData(_sound, 0);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
			result = FMOD_Sound_ReadData(_sound, buffer, _soundLength * bytesPerSample() * channels, &read);  UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

			UDEBUG("_soundLength * bytesPerSample() * channels=%d", _soundLength * bytesPerSample() * channels);
			if(read)
			{
				UDEBUG("read=%d", read);
				// push back in the frames buffer
				pushBackSamples(buffer, read);
			}

			delete[] buffer;
		}
	}
	return ok;
}

void UAudioRecorderFile::mainLoopBegin()
{
	// Set paused flag
	if(_channel)
	{
		FMOD_RESULT result;
		result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		result = FMOD_Channel_SetPaused(_channel, false);
		if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
		{
			UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		}
	}

	if(!_sound)
	{
		UERROR("Recorder is not initialized.");
		this->kill();
		return;
	}

	if(!_playSoundWhileRecording)
	{
		this->kill();
		return;
	}
}

void UAudioRecorderFile::mainLoop()
{
	FMOD_RESULT result;

	void *ptr1, *ptr2;
	unsigned int blockLength;
	unsigned int len1, len2;
	unsigned int recordPos = 0;

	result = FMOD_Channel_GetPosition(_channel, &recordPos, FMOD_TIMEUNIT_PCM);
	if(result != FMOD_OK)
	{
		// invalid handle means the channel is stopped
		UDEBUG("PlayerRecorder: Recording reached the end of file...");
		this->kill();
		return;
	}

	if (recordPos != _lastRecordPos)
	{
		blockLength = recordPos - _lastRecordPos;

		// * exinfo.numchannels * 2 = stereo 16bit.  1 sample = 4 bytes.
		// Lock the sound to get access to the raw data.
		FMOD_Sound_Lock(_sound, _lastRecordPos * channels() * bytesPerSample(), blockLength * channels() * bytesPerSample(), &ptr1, &ptr2, &len1, &len2);
		{
			if (ptr1 && len1)
			{
				// push back in the frames buffer
				pushBackSamples(ptr1, len1);
			}

			if (ptr2 && len2)
			{
				// push back in the frames buffer
				pushBackSamples(ptr2, len2);
			}
		}
		//Unlock the sound to allow FMOD to use it again.
		FMOD_Sound_Unlock(_sound, ptr1, ptr2, len1, len2);

		_lastRecordPos = recordPos;
	}

	UAudioSystem::update();

	uSleep(10);
}

void UAudioRecorderFile::mainLoopEnd()
{
	// Set paused flag
	if(_channel)
	{
		FMOD_RESULT result;
		result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		result = FMOD_Channel_SetPaused(_channel, true);
		if ((result != FMOD_OK) && (result != FMOD_ERR_INVALID_HANDLE) && (result != FMOD_ERR_CHANNEL_STOLEN))
		{
			UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		}
	}
	UAudioRecorder::mainLoopEnd();
}
