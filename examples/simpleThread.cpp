#include <utilite/UThreadNode.h>
#include <stdio.h>

/**
 * This is a simple example on how to use the UThreadNode class.
 * The output of this program is :
 * @code
 * 		Example with the UThreadNode class
 * 		This is called once before entering the main thread loop.
 * 		This is the loop 1...
 * 		This is the loop 2...
 * 		This thread will now wait on a semaphore...
 * 		Releasing the semaphore...
 * 		Thread woke up!
 * 		SimpleThread destructor
 * @endcode
 */

class SimpleThread : public UThreadNode
{
public:
	SimpleThread() : loopCount_(0) {}
	~SimpleThread() {
		printf("SimpleThread destructor\n");
		this->join(this);
	}

protected:
	virtual void startInit() {
		printf("This is called once before entering the main thread loop.\n");
		loopCount_ = 1;
	}

	virtual void mainLoop() {
		if(loopCount_ < 3) {
			printf("This is the loop %d...\n", loopCount_++);
			uSleep(10);
		}
		else {
			printf("This thread will now wait on a semaphore...\n");
			semaphore_.acquire();
			printf("Thread woke up!\n");
		}
	}

	virtual void killCleanup() {
		printf("Releasing the semaphore...\n");
		semaphore_.release();
	}
private:
	int loopCount_;
	USemaphore semaphore_;
};

int main(int argc, char * argv[])
{
	SimpleThread t;
	printf("Example with the UThreadNode class\n\n");
	t.start();
	uSleep(100);
	t.kill();
	return 0;
}

