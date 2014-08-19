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

/*
 * Originally written by Phillip Sitbon
 *  Copyright 2003
 */

#ifndef USEMAPHORE_H
#define USEMAPHORE_H

#include <errno.h>

#ifdef WIN32
#include "utilite/Win32/UWin32.h"
#define SEM_VALUE_MAX ((int) ((~0u) >> 1))
#else
#include <pthread.h>
#endif

/**
 * A semaphore class.
 *
 * On an acquire() call, the calling thread is blocked if the
 * USemaphore's value is <= 0. It is unblocked when release() is called.
 * The function acquire() decreases by 1 (default) the
 * semaphore's value and release() increases it by 1 (default).
 *
 * Example:
 * @code
 * USemaphore s;
 * s.acquire(); // Will wait until s.release() is called by another thread.
 * @endcode
 *
 * @see UMutex
 */
class USemaphore
{
public:
	/**
	 * The constructor. The semaphore waits on acquire() when its value is <= 0.
	 * @param n number to initialize
	 */
	USemaphore( int initValue = 0 )
	{
#ifdef WIN32
		S = CreateSemaphore(0,initValue,SEM_VALUE_MAX,0);
#else
		_available = initValue;
		pthread_mutex_init(&_waitMutex, NULL);
		pthread_cond_init(&_cond, NULL);
#endif
	}

	virtual ~USemaphore()
	{
#ifdef WIN32
		CloseHandle(S);
#else
		pthread_cond_destroy(&_cond);
		pthread_mutex_destroy(&_waitMutex);
#endif
	}

	/**
	 * Acquire the semaphore. If semaphore's value is <=0, the
	 * calling thread will wait until the count acquired is released.
	 * @see release()
	 * @param n number to acquire
	 */
#ifdef WIN32
	void acquire(int n = 1) const
	{
		while(n-- > 0)
		{
			WaitForSingleObject((HANDLE)S,INFINITE);
		}
#else
	void acquire(int n = 1)
	{
		pthread_mutex_lock(&_waitMutex);
		while (n > _available)
		{
			while(1)
			{
				pthread_cond_wait(&_cond, &_waitMutex);
				break;
			}
		}
		_available -= n;
		pthread_mutex_unlock(&_waitMutex);
#endif
	}

	/*
	 * Try to acquire the semaphore, not a blocking call.
	 * @return false if the semaphore can't be taken without waiting (value <= 0), true otherwise
	 */
#ifdef WIN32
	int acquireTry() const
	{
		return ((WaitForSingleObject((HANDLE)S,INFINITE)==WAIT_OBJECT_0)?0:EAGAIN);
#else
	int acquireTry(int n)
	{
		pthread_mutex_lock(&_waitMutex);
		if(n > _available)
		{
			pthread_mutex_unlock(&_waitMutex);
			return false;
		}
		_available -= n;
		pthread_mutex_unlock(&_waitMutex);
		return true;
#endif
	}

	/**
	 * Release the semaphore, increasing its value by 1 and
	 * signaling waiting threads (which called acquire()).
	 */
#ifdef WIN32
	int release(int n = 1) const
	{
		return (ReleaseSemaphore((HANDLE)S,n,0)?0:ERANGE);
#else
	void release(int n = 1)
	{
		pthread_mutex_lock(&_waitMutex);
		_available += n;
		pthread_cond_broadcast(&_cond);
		pthread_mutex_unlock(&_waitMutex);
#endif
	}

	/**
	 * Get the USempahore's value.
	 * @return the semaphore's value
	 */
#ifdef WIN32
	int value() const
	{
		LONG V = -1; ReleaseSemaphore((HANDLE)S,0,&V); return V;
#else
	int value()
	{
		int value = 0;
		pthread_mutex_lock(&_waitMutex);
		value = _available;
		pthread_mutex_unlock(&_waitMutex);
		return value;
#endif
	}

#ifdef WIN32
	/*
	 * Reset the semaphore count.
	 * @param init the initial value
	 * TODO implement on posix ?
	 */
	void reset( int init = 0 )
	{
		CloseHandle(S);
		S = CreateSemaphore(0,init,SEM_VALUE_MAX,0);
	}
#endif

private:
#ifdef WIN32
	HANDLE S;
#else
	pthread_mutex_t _waitMutex;
	pthread_cond_t _cond;
	int _available;
#endif
	void operator=(const USemaphore &S){}
	USemaphore(const USemaphore &S){}
};

#endif // USEMAPHORE_H
