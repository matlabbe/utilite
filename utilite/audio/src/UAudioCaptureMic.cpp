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

#include "utilite/UAudioCaptureMic.h"
#ifdef BUILT_WITH_LAME
#include "utilite/UMp3Encoder.h"
#endif
#include "UAudioSystem.h"
#include "utilite/UWav.h"
#include <utilite/UFile.h>
#include <utilite/ULogger.h>
#include <string.h>

UAudioCaptureMic::UAudioCaptureMic(int driver,
						 int fs,
						 int frameLength,
						 int bytesPerSample,
						 int channels) :
	UAudioCapture(fs, frameLength, bytesPerSample, channels),
	_driver(driver),
	_fileName(""),
	_maxFileSize(0),
	_encodeToMp3(false),
	_fp(0),
	_sound(0),
	_dataLength(0),
	_soundLength(0),
	_lastRecordPos(0)

{
    UDEBUG("");
}

UAudioCaptureMic::UAudioCaptureMic(const std::string &fileName,
                         int maxFileSizeMb,
                         int driver,
                         int fs,
						 int frameLength,
						 int bytesPerSample,
						 int channels) :
	UAudioCapture(fs, frameLength, bytesPerSample, channels),
	_driver(driver),
	_fileName(fileName),
	_maxFileSize(maxFileSizeMb*1000000), // Mb to bytes
	_encodeToMp3(false),
	_fp(0),
	_sound(0),
	_dataLength(0),
	_soundLength(0),
	_lastRecordPos(0)
{
    UDEBUG("");
}

UAudioCaptureMic::~UAudioCaptureMic()
{
    this->join(this);
    this->close();
}

void UAudioCaptureMic::close()
{
	if(_sound)
	{
		if(_fp)
		{
			// Write back the wav header now that we know its length.
			int channels, bits;
			float rate;
			FMOD_Sound_GetFormat(_sound, 0, 0, &channels, &bits);
			FMOD_Sound_GetDefaults(_sound, &rate, 0, 0, 0);
			UWav::writeWavHeader(_fp, _dataLength, rate, channels, bits);
			fclose(_fp);
			_fp = 0;

			if(_encodeToMp3)
			{
#ifdef BUILT_WITH_LAME
				// Encode to mp3
				UMp3Encoder mp3Encoder;

				// Rename the wav file
				std::string tempFileName = _fileName;
				tempFileName.append("TMP");
				UFile::rename(_fileName, tempFileName);

				if(mp3Encoder.encode(tempFileName, _fileName) == 0)
				{
					//Erase the wav file
					UFile::erase(tempFileName);
				}
#endif
			}
		}
		FMOD_RESULT result;
		result = FMOD_Sound_Release(_sound); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		_sound = 0;
	}
	if(_fp)
	{
		fclose(_fp);
		_fp = 0;
	}
}

bool UAudioCaptureMic::init()
{
	this->close();
	bool ok = UAudioCapture::init();
	if(ok)
	{
		std::string::size_type loc;

		if(_fileName.size())
		{
			loc = _fileName.find( ".mp3", 0 );
			if( loc != std::string::npos )
			{
#ifdef BUILT_WITH_LAME
				_encodeToMp3 = true;
#else
				_fileName.append(".wav");
				UERROR("Cannot write to a mp3, saving to a wav instead (%s)", _fileName.c_str());
#endif
			}
			_fp = fopen(_fileName.c_str(), "wb");
		}

		FMOD_RESULT result;
		FMOD_BOOL isRecording = false;
		result = UAudioSystem::isRecording(_driver, &isRecording); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		if(isRecording)
		{
			result = UAudioSystem::recordStop(_driver); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		}

		_dataLength = 0;
		_soundLength = 0;
		_lastRecordPos = 0;
		FMOD_CREATESOUNDEXINFO exinfo;
		memset(&exinfo, 0, sizeof(FMOD_CREATESOUNDEXINFO));
		exinfo.cbsize           = sizeof(FMOD_CREATESOUNDEXINFO);
		exinfo.numchannels      = channels();
		if(bytesPerSample() == 1)
		{
			exinfo.format           = FMOD_SOUND_FORMAT_PCM8;
		}
		else if(bytesPerSample() == 2)
		{
			exinfo.format           = FMOD_SOUND_FORMAT_PCM16;
		}
		else if(bytesPerSample() == 3)
		{
			exinfo.format           = FMOD_SOUND_FORMAT_PCM24;
		}
		else if(bytesPerSample() == 4)
		{
			exinfo.format           = FMOD_SOUND_FORMAT_PCM32;
		}
		exinfo.defaultfrequency = (int)fs();
		exinfo.length           = exinfo.defaultfrequency * bytesPerSample() * exinfo.numchannels * 2; // 2 -> pour deux secondes

		result = UAudioSystem::createSound(0, FMOD_2D | FMOD_SOFTWARE | FMOD_OPENUSER, &exinfo, &_sound); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
		if(_fp)
		{
			int channels, bits;
			float rate;
			FMOD_Sound_GetFormat(_sound, 0, 0, &channels, &bits);
			FMOD_Sound_GetDefaults(_sound, &rate, 0, 0, 0);
			UWav::writeWavHeader(_fp, _dataLength, rate, channels, bits);        //    Write out the wav header.  La longueur sera de 0 puisqu'elle est incunnue pour l'instant.
		}

		result = FMOD_Sound_GetLength(_sound, &_soundLength, FMOD_TIMEUNIT_PCM); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
	}
	return ok;
}

std::vector<std::string> UAudioCaptureMic::getRecordDrivers()
{
    /*
        Enumerate record devices
    */
    FMOD_RESULT result;
    int numdrivers = 0;
    std::vector<std::string> listDrivers;

    result = UAudioSystem::getRecordNumDrivers(&numdrivers); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    for (int count=0; count < numdrivers; count++)
    {
        char name[256] = {0};

        result = UAudioSystem::getRecordDriverInfo(count, name, 256, 0); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

        listDrivers.push_back(name);
    }
    return listDrivers;
}

void UAudioCaptureMic::mainLoopBegin()
{
	_lastRecordPos = 0;
	if(_sound)
	{
		FMOD_RESULT result;
		result = UAudioSystem::recordStart(_driver, _sound, true); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
	}
}

/*********************************
Fonction RecordFichier
Permet d'enregistrer un fichier WAV
*********************************/
void UAudioCaptureMic::mainLoop()
{
	if(!_sound)
	{
		UERROR("Recorder is not initialized.");
		this->kill();
		return;
	}
    FMOD_RESULT result;
    void *ptr1 = 0, *ptr2 = 0;
    int blockLength;
    unsigned int len1 = 0, len2 = 0;

    unsigned int recordPos = 0;

	result = UAudioSystem::getRecordPosition(_driver, &recordPos); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
	if (recordPos != _lastRecordPos)
	{
		blockLength = (int)recordPos - (int)_lastRecordPos;
		if (blockLength < 0)
		{
			blockLength += _soundLength;
		}

		// * exinfo.numchannels * 2 = stereo 16bit.  1 sample = 4 bytes.
		// Lock the sound to get access to the raw data.
		FMOD_Sound_Lock(_sound, _lastRecordPos * channels() * bytesPerSample(), blockLength * channels() * bytesPerSample(), &ptr1, &ptr2, &len1, &len2);
		{
			if (ptr1 && len1)    //    Write it to disk.
			{
				if(_fp)
				{
					//write to file
					_dataLength += fwrite(ptr1, 1, len1, _fp);
				}

				// push back in the frames buffer
				pushBackSamples(ptr1, len1);
			}

			if (ptr2 && len2)    //    Write it to disk.
			{
				if(_fp)
				{
					//write to file
					_dataLength += fwrite(ptr2, 1, len2, _fp);
				}

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

    // If we are recording to a file, make sure to stop 
    // when the maximum file size is reached
    if (_fp && _maxFileSize != 0 && int(_dataLength) + frameLength()*bytesPerSample() > _maxFileSize)
    {
        UWARN("Recording max memory reached (%d Mb)... stopped", _maxFileSize/1000000);
        this->kill();
    }
}

void UAudioCaptureMic::mainLoopEnd()
{
    UDEBUG("");
    FMOD_RESULT result;
    FMOD_BOOL isRecording;
    result = UAudioSystem::isRecording(_driver, &isRecording); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    if(isRecording)
    {
        result = UAudioSystem::recordStop(_driver); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    }
    UAudioCapture::mainLoopEnd();
}
