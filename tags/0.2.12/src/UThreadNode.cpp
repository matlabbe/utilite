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

#include "utilite/UThreadNode.h"
#include "utilite/ULogger.h"

////////////////////////////
// public:
////////////////////////////

UThreadNode::UThreadNode(Priority priority) : 
    state_(kSIdle), 
    priority_(priority), 
    handle_(0), 
    threadId_(0) 
{}

UThreadNode::~UThreadNode()
{
	ULOGGER_DEBUG("");
}

void UThreadNode::kill()
{
    ULOGGER_DEBUG("");
    killSafelyMutex_.lock();
    {
    	if(this->isRunning())
    	{
    		// Thread is creating
    		while(state_ == kSCreating || handle_ == 0 || threadId_ == 0)
			{
				uSleep(1);
			}

			if(state_ == kSRunning)
			{
				state_ = kSKilled;

				// Call function to do something before wait
				killCleanup();

				// No need to wait if the thread destroys itself
	#if WIN32
				if(UThread<void>::Self() != threadId_)
	#else
				if(UThread<void>::Self() != handle_)
	#endif
				{
					if(threadId_ && handle_)
					{
						ULOGGER_DEBUG("Thread %d joins the thread (%d)", UThread<void>::Self(), threadId_);
						THREAD_HANDLE h = handle_;
						int id = threadId_;
						killSafelyMutex_.unlock();
						if(this->Join(h))
						{
							ULOGGER_ERROR("Joining thread (%d) by %d failed", id, UThread<void>::Self());
						}
						killSafelyMutex_.lock();
					}
					else
					{
						ULOGGER_ERROR("This thread killed by %d is already dead?!? (threadId_=%d, handle_=%d)", UThread<void>::Self(), threadId_, handle_);
					}
				}
				else
				{
#if WIN32
					ULOGGER_DEBUG("killed itself (%d)", threadId_);
#else
					UThread<void>::Detach(handle_); // free pthread allocation
					ULOGGER_DEBUG("killed itself (%d)", handle_);
#endif
				}
			}
			else
			{
				UERROR("thread (%d) is supposed to be running...", threadId_);
			}
    	}
    	else
    	{
    		UDEBUG("thread (%d) is not running...", threadId_);
    	}
    }
    handle_ = 0;
    threadId_ = 0;
    killSafelyMutex_.unlock();
}

void UThreadNode::join()
{
	while(state_ == kSCreating)
	{
		uSleep(1);
	}
	runningMutex_.lock();
	runningMutex_.unlock();
}

void UThreadNode::start()
{
    ULOGGER_DEBUG("");

    if(state_ == kSIdle || state_ == kSKilled)// || state_ != StateKilled)
    {
        state_ = kSCreating;
        UThread<void>::Create(threadId_, &handle_);
        ULOGGER_DEBUG("StateThread::startThread() thread id=%d _handle=%d", threadId_, handle_);
        setPriority(priority_);
    }
}

//TODO : Support pThread
void UThreadNode::setPriority(Priority priority)
{
    if(handle_)
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
        SetThreadPriority(handle_, p);
#endif
    }
}

bool UThreadNode::isRunning() const
{
    return state_ == kSRunning || state_ == kSCreating;
}

bool UThreadNode::isIdle() const
{
    return state_ == kSIdle;
}

bool UThreadNode::isKilled() const
{
    return state_ == kSKilled;
}

////////////////////////////
// private:
////////////////////////////

void UThreadNode::ThreadMain()
{
	runningMutex_.lock();
    ULOGGER_DEBUG("");
    startInit();
    ULOGGER_DEBUG("Entering loop...");
    state_ = kSRunning;
    while(state_ == kSRunning)
    {
        mainLoop();
    }
    runningMutex_.unlock();
}
