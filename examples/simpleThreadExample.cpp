#include "UStateThread.h"
#include <stdio.h>

/**
 * This is a simple example on how to use the StateThread class.
 * The output of this program is :
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
 */

class SimpleThread : public UStateThread
{
public:
	SimpleThread() : _loopCount(0) {}
	~SimpleThread()
	{
		printf("SimpleThread destructor\n");
		this->killSafely();
	}

protected:
	virtual void threadBeforeLoop()
	{
		printf("This is called once before entering the main thread loop.\n");
		_loopCount = 1;
	}

	virtual void threadInnerLoop()
	{
		if(_loopCount < 3)
		{
			printf("This is the loop %d...\n", _loopCount++);
			SLEEP(10);
		}
		else
		{
			printf("This thread will now wait on a semaphore...\n");
			_aSemaphore.acquire();
			printf("Thread woke up!\n");
		}
	}

	virtual void killSafelyInner()
	{
		printf("Releasing the semaphore...\n");
		_aSemaphore.acquire();
	}
private:
	int _loopCount;
	USemaphore _aSemaphore;
};

int main(int argc, char * argv[])
{
	SimpleThread t;
	printf("Example with the StateThread class\n\n");
	t.startThread();
	SLEEP(100);
	t.killSafely();
	return 0;
}

