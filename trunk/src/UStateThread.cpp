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

#include "UStateThread.h"
#include "ULogger.h"

////////////////////////////
// public:
////////////////////////////

UStateThread::UStateThread(Priority priority) : 
    _state(kSIdle), 
    _priority(priority), 
    _handle(0), 
    _threadId(0) 
{}

UStateThread::~UStateThread()
{
	LOGGER_DEBUG("");
}

void UStateThread::killSafely()
{
    LOGGER_DEBUG("");
    _killSafelyMutex.Lock();
    {
    	if(_state == kSCreating)
		{
    		LOGGER_DEBUG("thread is creating...(caller %d is waiting)", UThread<void>::Self());
			while(_state == kSCreating)
			{
				SLEEP(10);
			}
		}

        if(_state == kSRunning)
        {
            _state = kSKilled;

            // Call function to do something before wait
            killSafelyInner();

            // No need to wait if the thread destroys itself
#if WIN32
            if(UThread<void>::Self() != _threadId)
#else
            if(UThread<void>::Self() != _handle)
#endif
            {
            	if(_threadId && _handle)
            	{
					LOGGER_DEBUG("Thread %d joins the thread (%d)", UThread<void>::Self(), _threadId);
					THREAD_HANDLE h = _handle;
					int id = _threadId;
					_killSafelyMutex.Unlock();
					if(this->Join(h))
					{
						LOGGER_ERROR("Joining thread (%d) by %d failed", id, UThread<void>::Self());
					}
					_killSafelyMutex.Lock();
            	}
            	else
            	{
            		LOGGER_ERROR("This thread killed by %d is already dead?!?", UThread<void>::Self());
            	}
            }
            else
            {
#if WIN32
            	LOGGER_DEBUG("killed itself (%d)", _threadId);
#else
            	UThread<void>::Detach(_handle); // free pthread allocation
            	LOGGER_DEBUG("killed itself (%d)", _handle);
#endif
            }
        }
        else
        {
        	LOGGER_DEBUG("thread (%d) not running...", _threadId);
        }
    }
    _handle = 0;
    _threadId = 0;
    _killSafelyMutex.Unlock();
}

void UStateThread::startThread()
{
    LOGGER_DEBUG("");

    if(_state == kSIdle || _state == kSKilled)// || _state != StateKilled)
    {
        _state = kSCreating;
        UThread<void>::Create(_threadId, &_handle);
        LOGGER_DEBUG("StateThread::startThread() thread id=%d _handle=%d", _threadId, _handle);
        setThreadPriority(_priority);
    }
}

//TODO : Support pThread
void UStateThread::setThreadPriority(Priority priority)
{
    if(_handle)
    {
#ifdef WIN32
        int p = THREAD_PRIORITY_NORMAL;
        switch(priority)
        {
            case kPLow:
                p = THREAD_PRIORITY_LOWEST;
                break;

            case kPBelowNormal:
                p = THREAD_PRIORITY_BELOW_NORMAL;
                break;

            case kPNormal:
                p = THREAD_PRIORITY_NORMAL;
                break;

            case kPAboveNormal:
                p = THREAD_PRIORITY_ABOVE_NORMAL;
                break;

            case kPRealTime:
                p = THREAD_PRIORITY_TIME_CRITICAL;
                break;

            default:
                break;
        }
        SetThreadPriority(_handle, p);
#endif
    }
}

bool UStateThread::isRunning() const
{
    return _state == kSRunning || _state == kSCreating;
}

bool UStateThread::isIdle() const
{
    return _state == kSIdle;
}

bool UStateThread::isKilled() const
{
    return _state == kSKilled;
}

////////////////////////////
// private:
////////////////////////////

void UStateThread::ThreadMain()
{
    LOGGER_DEBUG("");
    threadBeforeLoop();
    LOGGER_DEBUG("Entering loop...");
    _state = kSRunning;
    while(_state == kSRunning)
    {
        threadInnerLoop();
    }
}
