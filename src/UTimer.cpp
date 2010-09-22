/**
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "UTimer.h"
#include "ULogger.h"

///////////////////////
// public:
///////////////////////
UTimer::UTimer()
{
#ifdef WIN32
    QueryPerformanceFrequency(&_frequency);
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
    QueryPerformanceCounter(&_startTimeRecorded);
    _stopTimeRecorded = _startTimeRecorded;
}
void UTimer::stop()
{
    QueryPerformanceCounter(&_stopTimeRecorded);

}
double UTimer::getElapsedTime()
{
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);
    return double(now.QuadPart - _startTimeRecorded.QuadPart) / _frequency.QuadPart;
}
double UTimer::getInterval()
{
    return double(_stopTimeRecorded.QuadPart - _startTimeRecorded.QuadPart) / _frequency.QuadPart;
}
#else
double UTimer::now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return double(tv.tv_sec + tv.tv_usec) / 1000000;
}

void UTimer::start()
{
    gettimeofday(&_startTimeRecorded, NULL);
    _stopTimeRecorded = _startTimeRecorded;
}
void UTimer::stop()
{
    gettimeofday(&_stopTimeRecorded, NULL);

}
double UTimer::getElapsedTime()
{
	return UTimer::now() - double(_startTimeRecorded.tv_sec) + double(_startTimeRecorded.tv_usec) / 1000000;

}
double UTimer::getInterval()
{
	double start = double(_startTimeRecorded.tv_sec) + double(_startTimeRecorded.tv_usec) / 1000000;
	double stop = double(_stopTimeRecorded.tv_sec) + double(_stopTimeRecorded.tv_usec) / 1000000;
	return stop - start;
}
#endif

double UTimer::ticks()        // Stop->start and return Interval
{
    stop();
    double inter = getInterval();
    start();
    return inter;
}
