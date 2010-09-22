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

#ifndef STATETHREAD_H
#define STATETHREAD_H

#include "UtiLiteExp.h" // DLL export/import defines

#include "UThread.h"

/**
 * This class is a thread with states. There are 3 types of
 * state (IDLE, RUNNING, KILLED). Use the method startThread() to
 * start the thread (IDLE -> RUNNING). To kill the thread safely,
 * use killSafely() (RUNNING -> KILLED). The use of kill() is not safe.
 * It is recommended to use killSafely() to let the thread finishes his
 * loop so it can unlocks Mutex or Semaphore previously locked to avoid
 * deadlocks(when an other thread wait for a resource locked by this thread).
 *
 * For most of inherited classes, only threadInnerLoop() needs to be implemented.
 *
 * Example:
 * @code
 * 		#include "StateThread.h"
 * 		#include <stdio.h>
 * 		class SimpleThread : public Util::StateThread
 * 		{
 * 			public:
 * 				SimpleThread() : _loopCount(0) {}
 * 				~SimpleThread()
 * 				{
 * 					printf("SimpleThread destructor\n");
 * 					this->killSafely();
 * 				}
 *
 * 			protected:
 * 				virtual void threadBeforeLoop()
 * 				{
 * 					printf("This is called once before entering the main thread loop.\n");
 * 					_loopCount = 1;
 * 				}
 *
 * 				virtual void threadInnerLoop()
 * 				{
 * 					if(_loopCount < 3)
 * 					{
 * 						printf("This is the loop %d...\n", _loopCount++);
 * 						SLEEP(10);
 * 					}
 * 					else
 * 					{
 * 						printf("This thread will now wait on a semaphore...\n");
 * 						_aSemaphore.Wait();
 * 						printf("Thread woke up!\n");
 * 					}
 * 				}
 *
 * 				virtual void killSafelyInner()
 * 				{
 *					printf("Releasing the semaphore...\n");
 * 					_aSemaphore.Post();
 * 				}
 * 			private:
 * 				int _loopCount;
 * 				Util::Semaphore _aSemaphore;
 * 		};
 *
 * 		int main(int argc, char * argv[])
 * 		{
 *			SimpleThread t;
 *			printf("Example with the StateThread class\n");
 *			t.startThread();
 *			SLEEP(100);
 *			t.killSafely();
 * 			return 0;
 * 		}
 * @endcode
 * The output is :
 * @code
 * 		Example with the StateThread class
 * 		This is called once before entering the main thread loop.
 * 		This is the loop 1...
 * 		This is the loop 2...
 * 		This thread will now wait on a semaphore...
 * 		Releasing the semaphore...
 * 		Thread woke up!
 * 		SimpleThread destructor
 * @endcode
 *
 * @see startThread()
 * @see killSafely()
 * @see threadInnerLoop()
 *
 * TODO Add a join() method to be able to wait for the termination of a thread.
 */
class UTILITE_EXP UStateThread : public UThread<void>
{
public:
    /**
     * Enum of priorities
     */
    enum Priority{kPLow, kPBelowNormal, kPNormal, kPAboveNormal, kPRealTime};

public:
    /**
     * The constructor.
     */
    UStateThread(Priority priority = kPNormal);

    /**
     * The destructor. Inherited classes must call killSafely() to avoid memory leaks...
     */
    virtual ~UStateThread();

    /**
     * It kills the thread safely. It lets the thread finishes his
     * loop so it can unlocks Mutex or Semaphore previously locked to avoid
     * deadlocks(when an other thread wait for a ressource locked by this thread).
     * This functions does nothing if the thread is not started or is killed.
     */
    void killSafely();

    /**
     * Start the thread. Once the thread is started, subsequent calls
     * to startThread() are ignored until the thread is killed.
     * @see killSafely()
     */
    void startThread();


    /**
     * Set the thread priority.
     * @param priority the priority
     * @todo : Support pThread
     */
    void setThreadPriority(Priority priority);

    bool isRunning() const;
    bool isIdle() const;
    bool isKilled() const;


protected:

private:

    /**
     * Inherited method ThreadMain() from Thread.
     * @see Thread<void>
     */
    virtual void ThreadMain();

    /**
     * Inherited method Create() from Thread.
     * Here we force this function to be private so the
     * inherited class can't have access to it.
     * @see Thread<void>
     */
    int Create(
      Handle  * const     & H               = 0,
      const bool          & CreateDetached  = false,
      const unsigned int  & StackSize       = 0,
      const bool          & CancelEnable    = false,   // UNUSED
      const bool          & CancelAsync     = false    // UNUSED
    ) const;

    /**
     * Inherited method Join() from Thread.
     * Here we force this function to be private so the
     * inherited class can't have access to it.
     * @see Thread<void>
     */
    int Join( const Handle &H ) {return UThread<void>::Join(H);}

    /**
	 * Inherited method Kill() from Thread.
	 * Here we force this function to be private so the
	 * inherited class can't have access to it.
	 * @see Thread<void>
	 */
	int Kill( const Handle &H ) {return UThread<void>::Kill(H);}

	/**
	 * Inherited method Detach() from Thread.
	 * Here we force this function to be private so the
	 * inherited class can't have access to it.
	 * @see Thread<void>
	 */
	int Detach( const Handle &H ) {return UThread<void>::Detach(H);}

    /**
     * Virtual method threadBeforeLoop().
     * User can implement this function to add a behavior
     * before the main loop when the thread is started. It is
     * called once.
     */
    virtual void threadBeforeLoop() {}

    /**
     * Pure virtual method threadInnerLoop().
     * The inner loop of the thread. This method is called
     * in the main loop of the thread while the thread is running.
     *
     * @see ThreadMain()
     */
    virtual void threadInnerLoop() = 0;

    /**
     * Virtual method killSafelyInner().
     * User can implement this function to add a behavior
     * before the thread is killed. It will wait for the loop to be finished. When this
     * function is called, the state of the thread is KILLED. It is useful to
     * wake up a sleeping thread to finish his loop and to avoid a deadlock.
     */
    virtual void killSafelyInner() {}

private:
    enum State{kSIdle, kSCreating, kSRunning, kSKilled}; /**< Enum of states. */
    State _state; 			/**< The thread state. */
    Priority _priority; 	/**< The thread priority. */
    THREAD_HANDLE _handle; 	/**< The thread handle. */
#ifdef WIN32
    int _threadId; 			/**< The thread id. */
#else
    unsigned long _threadId; /**< The thread id. */
#endif
    UMutex _killSafelyMutex;	/**< Mutex used to protect the killSafely() method. */
};

#endif
