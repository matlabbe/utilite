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

#ifndef UMUTEX_H
#define UMUTEX_H

#include <errno.h>

#ifdef WIN32
  #include "utilite/Win32/UWin32.h"
#else
  #include <pthread.h>
#endif


/**
 * A mutex class.
 *
 * On a lock() call, the calling thread is blocked if the
 * UMutex was previously locked by another thread. It is unblocked when unlock() is called.
 *
 * On Unix (not yet tested on Windows), UMutex is recursive: the same thread can
 * call multiple times lock() without being blocked.
 *
 * Example:
 * @code
 * UMutex m; // Mutex shared with another thread(s).
 * ...
 * m.lock();
 * // Data is protected here from the second thread
 * //(assuming the second one protects also with the same mutex the same data).
 * m.unlock();
 *
 * @endcode
 *
 * @see USemaphore
 */
class UMutex
{

public:

	/**
	 * The constructor.
	 */
	UMutex()
	{
#ifdef WIN32
		InitializeCriticalSection(&C);
#else
		pthread_mutexattr_t attr;
		pthread_mutexattr_init(&attr);
		pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
		pthread_mutex_init(&M,&attr);
		pthread_mutexattr_destroy(&attr);
#endif
	}

	virtual ~UMutex()
	{
#ifdef WIN32
		DeleteCriticalSection(&C);
#else
		pthread_mutex_unlock(&M); pthread_mutex_destroy(&M);
#endif
	}

	/**
	 * Lock the mutex.
	 */
	int lock() const
	{
#ifdef WIN32
		EnterCriticalSection(&C); return 0;
#else
		return pthread_mutex_lock(&M);
#endif
	}

#ifdef WIN32
	#if(_WIN32_WINNT >= 0x0400)
	int lockTry() const
	{
		return (TryEnterCriticalSection(&C)?0:EBUSY);
	}
	#endif
#else
	int lockTry() const
	{
		return pthread_mutex_trylock(&M);
	}
#endif

	/**
	 * Unlock the mutex.
	 */
	int unlock() const
	{
#ifdef WIN32
		LeaveCriticalSection(&C); return 0;
#else
		return pthread_mutex_unlock(&M);
#endif
	}

	private:
#ifdef WIN32
		mutable CRITICAL_SECTION C;
#else
		mutable pthread_mutex_t M;
#endif
		void operator=(UMutex &M) {}
		UMutex( const UMutex &M ) {}
};

/**
 * Automatically lock the referenced mutex on constructor and unlock mutex on destructor.
 *
 * Example:
 * @code
 * UMutex m; // Mutex shared with another thread(s).
 * ...
 * int myMethod()
 * {
 *    UScopeMutex sm(m); // automatically lock the mutex m
 *    if(cond1)
 *    {
 *       return 1; // automatically unlock the mutex m
 *    }
 *    else if(cond2)
 *    {
 *       return 2; // automatically unlock the mutex m
 *    }
 *    return 0; // automatically unlock the mutex m
 * }
 *
 * @endcode
 *
 * @see UMutex
 */
class UScopeMutex
{
public:
	UScopeMutex(const UMutex & mutex) :
		mutex_(mutex)
	{
		mutex_.lock();
	}
	// backward compatibility
	UScopeMutex(UMutex * mutex) :
		mutex_(*mutex)
	{
		mutex_.lock();
	}
	~UScopeMutex()
	{
		mutex_.unlock();
	}
private:
	const UMutex & mutex_;
};

#endif // UMUTEX_H
