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

#define PRINT_DEBUG 0

void UThreadNode::join(UThreadNode * thread, bool killFirst)
{
	if(thread)
	{
		//make sure the thread is created
		while(thread->isCreating())
		{
			uSleep(1);
		}

#if WIN32
		if(PRINT_DEBUG)
			UDEBUG("Thread %d joining %d", UThread<void>::Self(), thread->getThreadId());
		if(UThread<void>::Self() == thread->getThreadId())
#else
		if(PRINT_DEBUG)
			UDEBUG("Thread %d joining %d", UThread<void>::Self(), thread->getThreadHandle());
		if(UThread<void>::Self() == thread->getThreadHandle())
#endif
		{
			UERROR("Thread cannot join itself");
			return;
		}

		if(killFirst)
		{
			thread->kill();
		}

		if(thread->getThreadHandle())
		{
			UThreadNode::Join(thread->getThreadHandle());
		}
		else if(PRINT_DEBUG)
		{
			UDEBUG("null thread");
		}
		if(PRINT_DEBUG)
			UDEBUG("Join ended for %d", UThread<void>::Self());
	}
}

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
	if(PRINT_DEBUG)
		ULOGGER_DEBUG("");
}

void UThreadNode::kill()
{
	if(PRINT_DEBUG)
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
			}
			else
			{
				UERROR("thread (%d) is supposed to be running...", threadId_);
			}
    	}
    	else
    	{
    		if(PRINT_DEBUG)
    			UDEBUG("thread (%d) is not running...", threadId_);
    	}
    }
    killSafelyMutex_.unlock();
}

void UThreadNode::start()
{
	if(PRINT_DEBUG)
		ULOGGER_DEBUG("");

    if(state_ == kSIdle || state_ == kSKilled)
    {
        state_ = kSCreating;
        UThread<void>::Create(threadId_, &handle_);
        if(PRINT_DEBUG)
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

bool UThreadNode::isCreating() const
{
    return state_ == kSCreating;
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
	if(PRINT_DEBUG)
		ULOGGER_DEBUG("");
    startInit();
    if(PRINT_DEBUG)
    	ULOGGER_DEBUG("Entering loop...");
    state_ = kSRunning;
    while(state_ == kSRunning)
    {
        mainLoop();
    }
    handle_ = 0;
    threadId_ = 0;
}
