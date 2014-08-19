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

#include "utilite/UAudioCaptureFile.h"
#include "UAudioSystem.h"
#include <utilite/UFile.h>
#include <utilite/ULogger.h>

UAudioCaptureFile::UAudioCaptureFile(const std::string &fileName, 
                           bool playSoundWhileRecording,
                           int frameLength) :
    UAudioCapture(0, frameLength, 0, 0),
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

UAudioCaptureFile::~UAudioCaptureFile()
{
    this->join(true);
    this->close();
}

void UAudioCaptureFile::close()
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

bool UAudioCaptureFile::init()
{
	this->close();
	bool ok = UAudioCapture::init();
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
				int soundLengthMs = (this->samples())/(this->channels()*this->bytesPerSample()*(this->fs()/1000));
				int min = (soundLengthMs/1000)/60;
				int sec = (soundLengthMs/1000) - min*60;
				int ms = soundLengthMs - sec*1000 - min*60*1000;
				UDEBUG("sound length = %d bytes / %d:%d.%d s", this->samples(), min, sec, ms);
			}

			delete[] buffer;
		}
	}
	return ok;
}

void UAudioCaptureFile::mainLoopBegin()
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

void UAudioCaptureFile::mainLoop()
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

void UAudioCaptureFile::mainLoopEnd()
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
	UAudioCapture::mainLoopEnd();
}

void UAudioCaptureFile::setPositionMs(unsigned int pos)
{
    UDEBUG("(%d)", pos);
    FMOD_RESULT result;
    result = UAudioSystem::update();         UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    //Pour modifier la position
    if(_channel)
    {
        // Get paused flag
    	FMOD_BOOL paused;
        result = FMOD_Channel_GetPaused(_channel, &paused);                    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        if(paused)
        {
            result = FMOD_Channel_SetPosition(_channel, pos, FMOD_TIMEUNIT_MS);    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            result = FMOD_Channel_GetPosition(_channel, &_lastRecordPos, FMOD_TIMEUNIT_PCM); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
        else
        {
            result = FMOD_Channel_SetPaused(_channel, true);                        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            result = FMOD_Channel_SetPosition(_channel, pos, FMOD_TIMEUNIT_MS);    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            result = FMOD_Channel_GetPosition(_channel, &_lastRecordPos, FMOD_TIMEUNIT_PCM); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
            result = FMOD_Channel_SetPaused(_channel, false);                    UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        }
    }
    else
    {
    	int posInSamples = 0;
    	if(pos>1000)
    	{
    		posInSamples = (this->fs() * int(pos/1000) * this->channels()*this->bytesPerSample());
    	}
    	posInSamples += (this->fs() * int(pos%1000) * this->channels()*this->bytesPerSample()) / 1000;

    	UDEBUG("posMs=%d posInSamples=%d totalSampples=%d", pos, posInSamples, this->samples());
    	if(this->samples() > posInSamples)
    	{
			this->removeSamples(0, posInSamples);
    	}
    	else
    	{
    		UWARN("Not enough samples (%d) to set position to %d ms (%d samples)", this->samples(), pos, posInSamples);
    	}
    }
}
