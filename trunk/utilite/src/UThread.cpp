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

#include "utilite/UThread.h"
#include "utilite/ULogger.h"
#ifdef __APPLE__
#include <mach/thread_policy.h>
#include <mach/mach.h>
#endif

#define PRINT_DEBUG 0

////////////////////////////
// public:
////////////////////////////

UThread::UThread(Priority priority) : 
    state_(kSIdle), 
    priority_(priority), 
    handle_(0), 
    threadId_(0),
    cpuAffinity_(-1)
{}

UThread::~UThread()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif
}

void UThread::kill()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif
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
#if PRINT_DEBUG
    		UDEBUG("thread (%d) is not running...", threadId_);
#endif
    	}
    }
    killSafelyMutex_.unlock();
}

void UThread::join(bool killFirst)
{
	//make sure the thread is created
	while(this->isCreating())
	{
		uSleep(1);
	}

#if WIN32
#if PRINT_DEBUG
	UDEBUG("Thread %d joining %d", UThreadC<void>::Self(), threadId_);
#endif
	if(UThreadC<void>::Self() == threadId_)
#else
#if PRINT_DEBUG
	UDEBUG("Thread %d joining %d", UThreadC<void>::Self(), handle_);
#endif
	if(UThreadC<void>::Self() == handle_)
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

#if PRINT_DEBUG
	UDEBUG("Join ended for %d", UThreadC<void>::Self());
#endif
}

void UThread::start()
{
#if PRINT_DEBUG
	ULOGGER_DEBUG("");
#endif

    if(state_ == kSIdle || state_ == kSKilled)
    {
    	if(state_ == kSKilled)
    	{
			// make sure it is finished
			runningMutex_.lock();
			runningMutex_.unlock();
    	}

        state_ = kSCreating;
        int r = UThreadC<void>::Create(threadId_, &handle_, true); // Create detached
        if(r)
        {
        	UERROR("Failed to create a thread! errno=%d", r);
        	threadId_=0;
        	handle_=0;
        	state_ = kSIdle;
        }
        else
        {
#if PRINT_DEBUG
        ULOGGER_DEBUG("StateThread::startThread() thread id=%d _handle=%d", threadId_, handle_);
#endif
        }
    }
}

//TODO : Support pThread
void UThread::setPriority(Priority priority)
{
	priority_ = priority;
}

//TODO : Support pThread
void UThread::applyPriority()
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

void UThread::setAffinity(int cpu)
{
	cpuAffinity_ = cpu;
	if(cpuAffinity_<0)
	{
		cpuAffinity_ = 0;
	}
}

//TODO : Support Windows and linux
void UThread::applyAffinity()
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

bool UThread::isCreating() const
{
    return state_ == kSCreating;
}

bool UThread::isRunning() const
{
    return state_ == kSRunning || state_ == kSCreating;
}

bool UThread::isIdle() const
{
    return state_ == kSIdle;
}

bool UThread::isKilled() const
{
    return state_ == kSKilled;
}

////////////////////////////
// private:
////////////////////////////

void UThread::ThreadMain()
{
	runningMutex_.lock();
	applyPriority();
	applyAffinity();

#if PRINT_DEBUG
	ULOGGER_DEBUG("before mainLoopBegin()");
#endif

	state_ = kSRunning;
    mainLoopBegin();

#if PRINT_DEBUG
	ULOGGER_DEBUG("before mainLoop()");
#endif

	while(state_ == kSRunning)
	{
		mainLoop();
	}

#if PRINT_DEBUG
	ULOGGER_DEBUG("before mainLoopEnd()");
#endif

	mainLoopEnd();

    handle_ = 0;
    threadId_ = 0;
    state_ = kSIdle;

    runningMutex_.unlock();
#if PRINT_DEBUG
	ULOGGER_DEBUG("Exiting thread loop");
#endif
}

