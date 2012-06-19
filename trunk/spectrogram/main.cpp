/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include <utilite/UFile.h>
#include <utilite/UMath.h>
#include <utilite/UStl.h>
#include <utilite/USpectrogram.h>
#include "utilite/UAudioRecorderFile.h"
#include "utilite/UAudioRecorderMic.h"
#include <fftw3.h>
#include <math.h>
#include <cmath>
#include <QtGui/QApplication>
#include <QtCore/QMetaType>
#include <utilite/UThreadNode.h>

void showUsage()
{
	printf("Usage:\n"
			"uspectrogram.exe [options] \"sound path\" or \"mic number\"\n"
			"  sound path :      Path of the WAV or MP3 file.\n"
			"  mic number :      Device id of the microphone (default: 0)\n."
			" Options:\n"
			"  -n # :             Window/frame length (default 1024, must be a multiple of 2).\n"
			"  -fs # :            Sampling frequency in Hz (default: 44100 Hz).\n"
			"  -offline :         Process entire file without playing it.\n"
			"  -no_win :          Don't use a window (default Hamming window of size \"n\").\n"
			"  -overlap :         Overlap frames by 50%%.\n"
			"  -debug :           Show debug traces.\n");
	exit(1);
}

class WorkerThread : public UThreadNode
{
public:
	WorkerThread(UAudioRecorder * recorder, std::vector<USpectrogram *> & spectrograms, bool hammingWindowUsed, bool overlap) :
		_recorder(recorder),
		_spectrograms(spectrograms),
		_hammingWindowUsed(hammingWindowUsed),
		_overlap(overlap)
	{
	}
	virtual ~WorkerThread() {}

protected:
	virtual void mainLoop()
	{
		if(!_recorder || _spectrograms.size()!=(unsigned int)_recorder->channels())
		{
			UERROR("_spectrograms.size()=%d, _recorder->channels()=%d", _spectrograms.size(), _recorder->channels());
			this->kill();
			return;
		}

		int N = _overlap?_recorder->frameLength()*2:_recorder->frameLength();
		float in[N];
		fftwf_plan p;
		fftwf_complex * out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * N);
		p = fftwf_plan_dft_r2c_1d(N, in, out, 0);
		int frames = 0;
		std::vector<float> window;
		if(_hammingWindowUsed)
		{
			window = uHamming(N);
		}
		else
		{
			window = std::vector<float>(N, 1.0f);
		}

		std::vector<char> acquiredFrame;
		std::vector<char> frame;
		while(_recorder->getNextFrame(acquiredFrame, true) &&
				acquiredFrame.size() == (unsigned int)_recorder->frameLength() * _recorder->bytesPerSample() * _recorder->channels() &&
				this->isRunning())
		{

			++frames;
			if(_spectrograms[0]->samplingRate())
			{
				printf("Processing frame %d (%.3f s)...\n", frames, float(frames*acquiredFrame.size()/(_recorder->bytesPerSample() * _recorder->channels()))/float(_spectrograms[0]->samplingRate()));
			}
			else
			{
				printf("Processing frame %d...\n", frames);
			}

			if(_overlap)
			{
				if(frame.size() == 0)
				{
					frame = acquiredFrame;
					continue;
				}
				else if(frame.size() == acquiredFrame.size())
				{
					frame.resize(frame.size()*2);
				}
				else
				{
					memcpy(frame.data(), frame.data() + frame.size()/2, frame.size()/2);
				}
				memcpy(frame.data(), acquiredFrame.data(), acquiredFrame.size());
			}
			else
			{
				frame = acquiredFrame;
			}

			int bytesPerSample = _recorder->bytesPerSample();
			int channels = _recorder->channels();

			for(int j=0; j<channels; ++j)
			{
				std::vector<float> v(frame.size()/(bytesPerSample*channels));
				for(unsigned int i=0; i<frame.size(); i+=bytesPerSample*channels)
				{
					if(bytesPerSample == 1)
					{
						v[i/(bytesPerSample*channels)] = window[i/(bytesPerSample*channels)] * ((float)(*(char*)&frame[i+bytesPerSample*j]) / float(1<<7)); // between 0 and 1
					}
					else if(bytesPerSample == 2)
					{
						v[i/(bytesPerSample*channels)] = window[i/(bytesPerSample*channels)] * ((float)(*(short*)&frame[i+bytesPerSample*j]) / float(1<<15)); // between 0 and 1
					}
					else if(bytesPerSample == 4)
					{
						v[i/(bytesPerSample*channels)] = window[i/(bytesPerSample*channels)] * ((float)(*(int*)&frame[i+bytesPerSample*j]) / float(1<<31)); // between 0 and 1
					}
				}
				memcpy(in, v.data(), N*sizeof(float));
				fftwf_execute(p); /* repeat as needed */
				std::vector<float> result(N/2+1);
				for(unsigned int i=0; i<result.size(); ++i)
				{
					float re = float(out[i][0]);
					float im = float(out[i][1]);
					result[i] = re*re+im*im; // squared magnitude
				}
				if(_spectrograms[j]->isVisible())
				{
					QMetaObject::invokeMethod(_spectrograms[j], "push", Qt::QueuedConnection, Q_ARG(std::vector<float>, result));
				}
				else
				{
					_spectrograms[j]->push(result);
				}
			}

		}
		fftwf_destroy_plan(p);
		fftwf_free(out);
		_recorder->join(true);
		this->kill();
	}
private:
	UAudioRecorder * _recorder;
	std::vector<USpectrogram *> _spectrograms;
	bool _hammingWindowUsed;
	bool _overlap;
};

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);

	std::string path;
	int frameLength = 1024;
	int fs = 44100;
	bool offline = false;
	bool hamming = true;
	int micDevice = -1;
	bool overlap = false;

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
		if(strcmp(argv[i], "-no_win") == 0)
		{
			hamming = false;
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

	QApplication app(argc, argv);
	ULogger::setType(ULogger::kTypeConsole);

	UAudioRecorder * recorder;
	if(micDevice >= 0 )
	{
		recorder = new UAudioRecorderMic("mic.wav", 100, micDevice, fs, overlap?frameLength/2:frameLength, 2, 1);
	}
	else
	{
		recorder = new UAudioRecorderFile(path, !offline, overlap?frameLength/2:frameLength);
	}
	recorder->init();
	recorder->start();

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

	std::vector<USpectrogram *> spectrograms(recorder->channels()); // one for each channel
	for(unsigned int i=0; i<spectrograms.size(); ++i)
	{
		spectrograms[i] = new USpectrogram(fs);
		spectrograms[i]->setMinimumSize(640, 480);
		spectrograms[i]->setWindowTitle(spectrograms[i]->windowTitle()+QString(" [Channel %1]").arg(i+1));
	}

	WorkerThread thread(recorder, spectrograms, hamming, overlap);
	thread.start();

	if(micDevice<0 && offline)
	{
		thread.join();
	}
	for(unsigned int i=0; i<spectrograms.size(); ++i)
	{
		if(offline)
		{
			spectrograms[i]->setScaled(true, true);
			spectrograms[i]->setOnlyLastFramesDrawn(false);
		}
		spectrograms[i]->show();
	}
	int r = app.exec();

	thread.join(true);
	delete recorder;

	for(unsigned int i=0; i<spectrograms.size(); ++i)
	{
		delete spectrograms[i];
	}

	return r;
}
