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

#ifndef TIMER_H
#define TIMER_H

#include "UtiLiteExp.h" // DLL export/import defines

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <time.h>
#endif

/**
 * This class is used to time some codes (in seconds).
 * On Windows, the performanceCounter is used.
 * Example:
 * @code
 *      Timer aTimer;
 *      timer.start();
 *      ... (do some work)
 *      timer.stop();
 *      int seconds = timer.getInterval();
 *      ...
 * @endcode
 */
class UTILITE_EXP UTimer
{
public:
    /** 
     * The constructor.
     */
    UTimer();

    /** 
     * The destructor.
     */
    ~UTimer();

    /** 
     * This method is used to get 
     * the time of the system right now.
     * @return double the time in seconds.
     */
    static double now();

    /** 
     * This method starts the timer.
     */
    void start();

    /** 
     * This method stops the timer.
     */
    void stop();

    /** 
     * This method is used to get the elapsed time
     * between now and the start().
     * @return double the interval in seconds.
     */
    double getElapsedTime();

    /**
	 * This method is used to get the interval time
	 * between stop() and the start().
	 * @return double the interval in seconds.
	 */
    double getInterval();

    /** 
     * This method is used to get the interval of 
     * the timer while it is running. It's automatically 
     * stop the timer, get the interval and restart 
     * the timer. It's the same of calling stop(), 
     * getInterval() and start().
     * @return double the interval in seconds.
     */
    double ticks();

private:
#ifdef WIN32
    LARGE_INTEGER _startTimeRecorded; /**< When we start the timer, timeRecorded is copied over lastTimeRecorded.*/
    LARGE_INTEGER _stopTimeRecorded;  /**< When we stop the timer. */

    LARGE_INTEGER _frequency;          /**< Keep the frequency of the counter */
#else
    struct timeval _startTimeRecorded; /**< When we start the timer, timeRecorded is copied over lastTimeRecorded.*/
    struct timeval _stopTimeRecorded;  /**< When we stop the timer. */
#endif
};

#endif //TIMER_H
