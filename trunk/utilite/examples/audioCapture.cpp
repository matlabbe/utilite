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

#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include <utilite/UFile.h>
#include <utilite/UMath.h>
#include <utilite/UStl.h>
#include <utilite/USpectrogram.h>
#include "utilite/UAudioCaptureFFT.h"
#include <math.h>
#include <cmath>
#include <QtGui/QApplication>
#include <QtCore/QMetaObject>
#include <utilite/UThread.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEventsManager.h>

void showUsage()
{
	printf("Usage:\n"
			"uspectrogram.exe [options] \"sound path\" or \"mic number\"\n"
			"  sound path :      Path of the WAV or MP3 file.\n"
			"  mic number :      Device id of the microphone (default: 0)\n."
			" Options:\n"
			"  -n # :             Window/frame length (default 4410, must be a multiple of 2).\n"
			"  -fs # :            Sampling frequency in Hz (default: 44100 Hz).\n"
			"  -offline :         Process entire file without playing it.\n"
			"  -no_win :          Don't use a window (default Hamming window of size \"n\").\n"
			"  -overlap :         Overlap frames by 50%%.\n"
			"  -debug :           Show debug traces.\n");
	exit(1);
}

class AudioFrameHandler : public UEventsHandler
{
public:
	AudioFrameHandler(bool offline, int channels, int fs) :
		_offline(offline)
	{
		//init
		for(int i=0; i<channels; ++i)
		{
			// one for each channel
			_spectrograms.push_back(new USpectrogram(fs));
			_spectrograms.back()->setMinimumSize(640, 480);
			_spectrograms.back()->setWindowTitle(_spectrograms.back()->windowTitle()+QString(" [Channel %1]").arg(i+1));
			if(_offline)
			{
				_spectrograms.back()->setAllDataKept(true);
			}
			else
			{
				_spectrograms.back()->show();
			}
		}
	}
	virtual ~AudioFrameHandler() {qDeleteAll(_spectrograms);}

protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("UAudioEvent") == 0)
		{
			UAudioEvent * aevent = (UAudioEvent*)event;
			if(aevent->type() == UAudioEvent::kTypeFrameFreqSqrdMagn)
			{
				const std::vector<std::vector<float> > & freq = aevent->frameFreq();
				for(unsigned int i=0; i<freq.size() && i<_spectrograms.size(); ++i)
				{
					QMetaObject::invokeMethod(_spectrograms[i], "push", Q_ARG(std::vector<float>, freq.at(i)));
				}
			}
			else if(aevent->type() == UAudioEvent::kTypeNoMoreFrames)
			{
				printf("no more frames...\n");

				if(_offline)
				{
					for(unsigned int i=0; i<_spectrograms.size(); ++i)
					{
						QMetaObject::invokeMethod(_spectrograms[i], "show");
					}
				}
			}
		}
	}

protected:
	bool _offline;
	std::vector<USpectrogram*> _spectrograms;
};

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);

	std::string path;
	int frameLength = 4410;
	int fs = 44100;
	bool offline = false;
	int micDevice = -1;
	bool overlap = false;

	if(argc < 2)
	{
		micDevice = 0;
	}
	else
	{
		for(int i=1; i<argc; ++i)
		{
			if(i == argc-1)
			{
				// The last must be the path
				path = argv[i];
				if(!(path.size() == 1 && uIsDigit(path.at(0))) != 0 && !UFile::exists(path.c_str()))
				{
					printf("Path not valid : %s\n", path.c_str());
					showUsage();
					exit(1);
				}
				else if(path.size() == 1 && uIsDigit(path.at(0)))
				{
					micDevice = std::atoi(path.c_str());
				}
				break;
			}
			if(strcmp(argv[i], "-n") == 0)
			{
				++i;
				if(i < argc)
				{
					frameLength = atoi(argv[i]);
					if(frameLength < 0)
					{
						printf("Frame length must be even and positive (%d)\n", frameLength);
						showUsage();
					}
				}
				else
				{
					showUsage();
				}
				continue;
			}
			if(strcmp(argv[i], "-fs") == 0)
			{
				++i;
				if(i < argc)
				{
					fs = atoi(argv[i]);
					if(fs < 0)
					{
						printf("Sampling frequency must be positive (%d)\n", fs);
						showUsage();
					}
				}
				else
				{
					showUsage();
				}
				continue;
			}
			if(strcmp(argv[i], "-offline") == 0)
			{
				offline = true;
				continue;
			}
			if(strcmp(argv[i], "-debug") == 0)
			{
				ULogger::setLevel(ULogger::kDebug);
				continue;
			}
			if(strcmp(argv[i], "-overlap") == 0)
			{
				overlap = true;
				continue;
			}

			printf("Unrecognized option : %s\n", argv[i]);
			showUsage();
		}
	}

	QApplication app(argc, argv);
	ULogger::setLevel(ULogger::kDebug);
	ULogger::setType(ULogger::kTypeConsole);


	UAudioCaptureFFT * recorder;
	if(micDevice >= 0 )
	{
		recorder = new UAudioCaptureFFT(UAudioEvent::kTypeFrameFreqSqrdMagn, micDevice, fs, frameLength, 1, 2, overlap);
	}
	else
	{
		recorder = new UAudioCaptureFFT(UAudioEvent::kTypeFrameFreqSqrdMagn, path, fs/frameLength, !offline, !offline, overlap);
	}
	recorder->init();


	fs = recorder->fs();

	if(micDevice>=0)
	{
		printf(" Using microphone %d\n", micDevice);
	}
	else
	{
		printf(" Path = %s\n", path.c_str());
	}
	printf(" Frame length = %d\n", frameLength);
	printf(" Sampling frequency = %d\n", fs);
	printf(" Offline = %s\n", offline?"true":"false");
	printf(" Channels = %d\n", recorder->channels());
	printf(" BytesPerSample = %d\n", recorder->bytesPerSample());
	printf(" Overlap = %s\n", overlap?"true":"false");

	AudioFrameHandler frameHandler(micDevice<0 && offline, recorder->channels(), recorder->fs());
	UEventsManager::addHandler(&frameHandler);


	recorder->start();

	printf("Processing...\n");

	int r = app.exec();

	UEventsManager::removeHandler(&frameHandler);
	delete recorder;

	return r;
}
