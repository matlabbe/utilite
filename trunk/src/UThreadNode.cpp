/*
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
#ifdef __APPLE__
#include <mach/thread_policy.h>
#include <mach/mach.h>
#endif

#define PRINT_DEBUG 0

////////////////////////////
// public:
////////////////////////////

UThreadNode::UThreadNode(Priority priority) : 
    state_(kSIdle), 
    priority_(priority), 
    handle_(0), 
    threadId_(0),
    cpuAffinity_(-1)
{}

UThreadNode::~UThreadNode()
{
	if(PRINT_DEBUG)
	{
		ULOGGER_DEBUG("");
	}
}

void UThreadNode::kill()
{
	if(PRINT_DEBUG)
	{
		ULOGGER_DEBUG("");
	}
    killSafelyMutex_.lock();
    {
    	if(this->isRunning())
    	{
    		// Thread is creating
    		while(state_ == kSCreating)
			{
				uSleep(1);
			}

			if(state_ == kSRunning)
			{
				state_ = kSKilled;

				// Call function to do something before wait
				mainLoopKill();
			}
			else
			{
				UERROR("thread (%d) is supposed to be running...", threadId_);
			}
    	}
    	else
    	{
    		if(PRINT_DEBUG)
    		{
    			UDEBUG("thread (%d) is not running...", threadId_);
    		}
    	}
    }
    killSafelyMutex_.unlock();
}

void UThreadNode::join(bool killFirst)
{
	//make sure the thread is created
	while(this->isCreating())
	{
		uSleep(1);
	}

#if WIN32
	if(PRINT_DEBUG)
	{
		UDEBUG("Thread %d joining %d", UThread<void>::Self(), threadId_);
	}
	if(UThread<void>::Self() == threadId_)
#else
	if(PRINT_DEBUG)
	{
		UDEBUG("Thread %d joining %d", UThread<void>::Self(), handle_);
	}
	if(UThread<void>::Self() == handle_)
#endif
	{
		UERROR("Thread cannot join itself");
		return;
	}

	if(killFirst)
	{
		this->kill();
	}

	runningMutex_.lock();
	runningMutex_.unlock();

	if(PRINT_DEBUG)
	{
		UDEBUG("Join ended for %d", UThread<void>::Self());
	}
}

void UThreadNode::start()
{
	if(PRINT_DEBUG)
	{
		ULOGGER_DEBUG("");
	}

    if(state_ == kSIdle || state_ == kSKilled)
    {
    	if(state_ == kSKilled)
    	{
			// make sure it is finished
			runningMutex_.lock();
			runningMutex_.unlock();
    	}

        state_ = kSCreating;
        UThread<void>::Create(threadId_, &handle_);
        if(PRINT_DEBUG)
        {
        	ULOGGER_DEBUG("StateThread::startThread() thread id=%d _handle=%d", threadId_, handle_);
        }
    }
}

//TODO : Support pThread
void UThreadNode::setPriority(Priority priority)
{
	priority_ = priority;
}

//TODO : Support pThread
void UThreadNode::applyPriority()
{
    if(handle_)
    {
#ifdef WIN32
        int p = THREAD_PRIORITY_NORMAL;
        switch(priority_)
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

void UThreadNode::setAffinity(int cpu)
{
	cpuAffinity_ = cpu;
	if(cpuAffinity_<0)
	{
		cpuAffinity_ = 0;
	}
}

//TODO : Support Windows and linux
void UThreadNode::applyAffinity()
{
	if(cpuAffinity_>0)
	{
#ifdef WIN32
#elif __APPLE__
		thread_affinity_policy_data_t affPolicy;
		affPolicy.affinity_tag = cpuAffinity_;
		kern_return_t ret = thread_policy_set(
				mach_thread_self(),
				THREAD_AFFINITY_POLICY,
				(integer_t*) &affPolicy,
				THREAD_AFFINITY_POLICY_COUNT);
		if(ret != KERN_SUCCESS)
		{
			UERROR("thread_policy_set returned %d", ret);
		}
#else
		/*unsigned long mask = cpuAffinity_;

		if (pthread_setaffinity_np(
			pthread_self(),
			sizeof(mask),
			&mask) <0)
		{
			UERROR("pthread_setaffinity_np failed");
		}
		}*/
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
	runningMutex_.lock();
	applyPriority();
	applyAffinity();

	if(PRINT_DEBUG)
	{
		ULOGGER_DEBUG("");
	}

	state_ = kSRunning;
    mainLoopBegin();

	if(PRINT_DEBUG)
	{
		ULOGGER_DEBUG("Entering loop...");
	}

	while(state_ == kSRunning)
	{
		mainLoop();
	}

    if(PRINT_DEBUG)
	{
		ULOGGER_DEBUG("");
	}
    handle_ = 0;
    threadId_ = 0;
    mainLoopEnd();
    state_ = kSIdle;
    runningMutex_.unlock();
}

// For backward compatibilities...
void UThreadNode::mainLoopBegin() {startInit();}
void UThreadNode::mainLoopKill() {killCleanup();}
