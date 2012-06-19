#include "utilite/UAudioRecorderMic.h"
#include "utilite/UMp3Encoder.h"
#include "System.h"
#include "utilite/UWav.h"
#include <utilite/UFile.h>
#include <utilite/ULogger.h>
#include <string.h>

UAudioRecorderMic::UAudioRecorderMic(int driver,
						 int fs,
						 int frameLength,
						 int bytesPerSample,
						 int channels) :
	UAudioRecorder(fs, frameLength, bytesPerSample, channels),
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

UAudioRecorderMic::UAudioRecorderMic(const std::string &fileName,
                         int maxFileSizeMb,
                         int driver,
                         int fs,
						 int frameLength,
						 int bytesPerSample,
						 int channels) :
	UAudioRecorder(fs, frameLength, bytesPerSample, channels),
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

UAudioRecorderMic::~UAudioRecorderMic()
{
    this->join(this);
    this->close();
}

void UAudioRecorderMic::close()
{
	if(_sound)
	{
		if(_fp)
		{
			// Write back the wav header now that we know its length.
			int channels, bits;
			float rate;
			_sound->getFormat(0, 0, &channels, &bits);
			_sound->getDefaults(&rate, 0, 0, 0);
			UWav::writeWavHeader(_fp, _dataLength, rate, channels, bits);
			fclose(_fp);
			_fp = 0;

			if(_encodeToMp3)
			{
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
			}
		}
		FMOD_RESULT result;
		result = _sound->release(); System::ERRCHECK(result);
		_sound = 0;
	}
	if(_fp)
	{
		fclose(_fp);
		_fp = 0;
	}
}

bool UAudioRecorderMic::init()
{
	this->close();
	bool ok = UAudioRecorder::init();
	if(ok)
	{
		std::string::size_type loc;

		if(_fileName.size())
		{
			loc = _fileName.find( ".mp3", 0 );
			if( loc != std::string::npos )
			{
				_encodeToMp3 = true;
			}
			_fp = fopen(_fileName.c_str(), "wb");
		}

		FMOD_RESULT result;
		bool isRecording = false;
		result = System::isRecording(_driver, &isRecording); System::ERRCHECK(result);
		if(isRecording)
		{
			result = System::recordStop(_driver); System::ERRCHECK(result);
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

		result = System::createSound(0, FMOD_2D | FMOD_SOFTWARE | FMOD_OPENUSER, &exinfo, &_sound); System::ERRCHECK(result);
		if(_fp)
		{
			int channels, bits;
			float rate;
			_sound->getFormat(0, 0, &channels, &bits);
			_sound->getDefaults(&rate, 0, 0, 0);
			UWav::writeWavHeader(_fp, _dataLength, rate, channels, bits);        //    Write out the wav header.  La longueur sera de 0 puisqu'elle est incunnue pour l'instant.
		}

		result = _sound->getLength(&_soundLength, FMOD_TIMEUNIT_PCM); System::ERRCHECK(result);
	}
	return ok;
}

std::vector<std::string> UAudioRecorderMic::getRecordDrivers()
{
    /*
        Enumerate record devices
    */
    FMOD_RESULT result;
    int numdrivers = 0;
    std::vector<std::string> listDrivers;

    result = System::getRecordNumDrivers(&numdrivers); System::ERRCHECK(result);
 
    for (int count=0; count < numdrivers; count++)
    {
        char name[256] = {0};

        result = System::getRecordDriverInfo(count, name, 256, 0); System::ERRCHECK(result);

        listDrivers.push_back(name);
    }
    return listDrivers;
}

void UAudioRecorderMic::mainLoopBegin()
{
	_lastRecordPos = 0;
	if(_sound)
	{
		FMOD_RESULT result;
		result = System::recordStart(_driver, _sound, true); System::ERRCHECK(result);
	}
}

/*********************************
Fonction RecordFichier
Permet d'enregistrer un fichier WAV
*********************************/
void UAudioRecorderMic::mainLoop()
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
    
	result = System::getRecordPosition(_driver, &recordPos); System::ERRCHECK(result);
	if (recordPos != _lastRecordPos)
	{
		blockLength = (int)recordPos - (int)_lastRecordPos;
		if (blockLength < 0)
		{
			blockLength += _soundLength;
		}

		// * exinfo.numchannels * 2 = stereo 16bit.  1 sample = 4 bytes.
		// Lock the sound to get access to the raw data.
		_sound->lock(_lastRecordPos * channels() * bytesPerSample(), blockLength * channels() * bytesPerSample(), &ptr1, &ptr2, &len1, &len2);
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
		_sound->unlock(ptr1, ptr2, len1, len2);

		_lastRecordPos = recordPos;
	}

    System::update();

    uSleep(10);

    // If we are recording to a file, make sure to stop 
    // when the maximum file size is reached
    if (_fp && _maxFileSize != 0 && int(_dataLength) + frameLength()*bytesPerSample() > _maxFileSize)
    {
        UWARN("Recording max memory reached (%d Mb)... stopped", _maxFileSize/1000000);
        this->kill();
    }
}

void UAudioRecorderMic::mainLoopEnd()
{
    UDEBUG("");
    FMOD_RESULT result;
    bool isRecording;
    result = System::isRecording(_driver, &isRecording); System::ERRCHECK(result);

    if(isRecording)
    {
        result = System::recordStop(_driver); System::ERRCHECK(result);
    }
    UAudioRecorder::mainLoopEnd();
}
