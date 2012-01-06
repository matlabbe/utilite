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

#ifndef UTHREADNODE_H
#define UTHREADNODE_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include "utilite/UThread.h"

/**
 * This class is a thread with states. There are 3 types of
 * state (IDLE, RUNNING, KILLED). Use the method start() to
 * start the thread (IDLE -> RUNNING). To kill the thread safely,
 * use kill() (RUNNING -> KILLED). The use of kill() is not safe.
 * It is recommended to use kill() to let the thread finishes his
 * loop so it can unlocks Mutex or Semaphore previously locked to avoid
 * deadlocks(when an other thread wait for a resource locked by this thread).
 *
 * For most of inherited classes, only mainLoop() needs to be implemented.
 *
 * Example:
 * @code
 * 		#include "utilite/UThreadNode.h"
 * 		#include <stdio.h>
 * 		class SimpleThread : public UThreadNode
 * 		{
 * 			public:
 * 				SimpleThread() : _loopCount(0) {}
 * 				~SimpleThread()
 * 				{
 * 					printf("SimpleThread destructor\n");
 * 					this->kill();
 * 				}
 *
 * 			protected:
 * 				virtual void startInit()
 * 				{
 * 					printf("This is called once before entering the main thread loop.\n");
 * 					_loopCount = 1;
 * 				}
 *
 * 				virtual void mainLoop()
 * 				{
 * 					if(_loopCount < 3)
 * 					{
 * 						printf("This is the loop %d...\n", _loopCount++);
 * 						uSleep(10);
 * 					}
 * 					else
 * 					{
 * 						printf("This thread will now wait on a semaphore...\n");
 * 						_aSemaphore.Wait();
 * 						printf("Thread woke up!\n");
 * 					}
 * 				}
 *
 * 				virtual void killCleanup()
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
 *			t.start();
 *			uSleep(100);
 *			t.kill();
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
 * @see start()
 * @see kill()
 * @see mainLoop()
 *
 */
class UTILITE_EXP UThreadNode : public UThread<void>
{
public:
    /**
     * Enum of priorities
     */
    enum Priority{kPLow, kPBelowNormal, kPNormal, kPAboveNormal, kPRealTime};

    /**
	 * The caller thread will wait until the thread has finished.
	 * TODO param timeout the maximum time to wait (0 is infinite)
	 * Note : blocking call
	 * @param thread thread to join.
	 * @param killFirst if you want kill() to be called before joining (default true), otherwise not.
	 */
	static void join(UThreadNode * thread, bool killFirst = true);

public:
    /**
     * The constructor.
     */
    UThreadNode(Priority priority = kPNormal);

    /**
     * The destructor. Inherited classes must call kill() to avoid memory leaks...
     */
    virtual ~UThreadNode();

    /**
     * Start the thread. Once the thread is started, subsequent calls
     * to start() are ignored until the thread is killed.
     * @see kill()
     */
    void start();

    /**
	 * It kills the thread safely. It lets the thread finishes his
	 * loop so it can unlocks Mutex or Semaphore previously locked to avoid
	 * deadlocks(when an other thread wait for a ressource locked by this thread).
	 * This functions does nothing if the thread is not started or is killed.
	 * Note : not a blocking call
	 */
	void kill();

    /**
     * Set the thread priority.
     * @param priority the priority
     */
    void setPriority(Priority priority);

    /**
	 * Set the thread affinity. This is applied during start of the thread.
	 * MAC OS X : http://developer.apple.com/library/mac/#releasenotes/Performance/RN-AffinityAPI/_index.html.
	 * @param cpu the cpu id (start at 1), 0 means no affinity (default).
	 */
	void setAffinity(int cpu = 0);

    bool isCreating() const;
    bool isRunning() const;
    bool isIdle() const;
    bool isKilled() const;

    Handle getThreadHandle() const {return handle_;}
#ifdef WIN32
    int getThreadId() const {return threadId_;}
#else
    unsigned long getThreadId() const {return threadId_;}
#endif


protected:

private:
    /**
	 * Virtual method startInit().
	 * User can implement this function to add a behavior
	 * before the main loop when the thread is started. It is
	 * called once.
	 */
	virtual void startInit() {}

	/**
	 * Pure virtual method mainLoop().
	 * The inner loop of the thread. This method is called
	 * in the main loop of the thread while the thread is running.
	 *
	 * @see mainLoop()
	 */
	virtual void mainLoop() = 0;

	/**
	 * Virtual method killCleanup().
	 * User can implement this function to add a behavior
	 * before the thread is killed. It will wait for the loop to be finished. When this
	 * function is called, the state of the thread is KILLED. It is useful to
	 * wake up a sleeping thread to finish his loop and to avoid a deadlock.
	 */
	virtual void killCleanup() {}

    /**
     * Inherited method ThreadMain() from Thread.
     * @see Thread<void>
     */
    void ThreadMain();

    /**
	 * Apply thread priority. This is called when starting the thread.
	 * *@todo : Support pthread
	 */
	void applyPriority();

	/**
	 * Apply cpu affinity. This is called when starting the thread.
	 * *@todo : Support Windows
	 */
	void applyAffinity();

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

    //Methods from UThread<void> class hided
    static int Join( Handle H )
	  { return UThread<void>::Join(H); }
	static int Kill( Handle H )
	  { return UThread<void>::Kill(H); }
	static int Detach( Handle H )
	  { return UThread<void>::Detach(H); }

private:
	void operator=(UThreadNode & t) {}
	UThreadNode( const UThreadNode & t ) {}

private:
    enum State{kSIdle, kSCreating, kSRunning, kSKilled}; /**< Enum of states. */
    State state_; 			/**< The thread state. */
    Priority priority_; 	/**< The thread priority. */
    Handle handle_; 	/**< The thread handle. */
#ifdef WIN32
    int threadId_; 			/**< The thread id. */
#else
    unsigned long threadId_; /**< The thread id. */
#endif
    int cpuAffinity_; /**< The cpu affinity. */
    UMutex killSafelyMutex_;	/**< Mutex used to protect the kill() method. */
};

#endif // UTHREADNODE_H
