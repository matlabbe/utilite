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

#include "utilite/UTimer.h"
#include "utilite/ULogger.h"

///////////////////////
// public:
///////////////////////
UTimer::UTimer()
{
#ifdef WIN32
    QueryPerformanceFrequency(&frequency_);
#endif
    start(); // This will initialize the private counters
}

UTimer::~UTimer() {}

#ifdef WIN32
double UTimer::now()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&count);
    return double(count.QuadPart) / freq.QuadPart;
}

void UTimer::start()
{
    QueryPerformanceCounter(&startTimeRecorded_);
    stopTimeRecorded_ = startTimeRecorded_;
}
void UTimer::stop()
{
    QueryPerformanceCounter(&stopTimeRecorded_);

}
double UTimer::getElapsedTime()
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
    return double(now.QuadPart - startTimeRecorded_.QuadPart) / frequency_.QuadPart;
}
double UTimer::getInterval()
{
	if(stopTimeRecorded_.QuadPart == startTimeRecorded_.QuadPart)
	{
		return getElapsedTime();
	}
	else
	{
		return double(stopTimeRecorded_.QuadPart - startTimeRecorded_.QuadPart) / frequency_.QuadPart;
	}
}
#else
double UTimer::now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;
}

void UTimer::start()
{
    gettimeofday(&startTimeRecorded_, NULL);
    stopTimeRecorded_ = startTimeRecorded_;
}
void UTimer::stop()
{
    gettimeofday(&stopTimeRecorded_, NULL);

}
double UTimer::getElapsedTime()
{
	return UTimer::now() - (double(startTimeRecorded_.tv_sec) + double(startTimeRecorded_.tv_usec) / 1000000.0);

}
double UTimer::getInterval()
{
	if(startTimeRecorded_.tv_sec == stopTimeRecorded_.tv_sec && startTimeRecorded_.tv_usec == stopTimeRecorded_.tv_usec)
	{
		return getElapsedTime();
	}
	else
	{
		double start = double(startTimeRecorded_.tv_sec) + double(startTimeRecorded_.tv_usec) / 1000000.0;
		double stop = double(stopTimeRecorded_.tv_sec) + double(stopTimeRecorded_.tv_usec) / 1000000.0;
		return stop - start;
	}
}
#endif

double UTimer::ticks()        // Stop->start and return Interval
{
    double inter = elapsed();
    start();
    return inter;
}
