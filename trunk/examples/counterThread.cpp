
#include "utilite/UThreadNode.h"
#include "utilite/UEventsHandler.h"
#include "utilite/UEventsManager.h"
#include "utilite/UEvent.h"

// Implement a simple event
class ResetEventt : public UEvent
{
public:
	ResetEventt() {}
	virtual ~ResetEventt() {}
	virtual std::string getClassName() const {return "ResetEventt";} // Must be implemented
};

// There is the thread counting indefinitely, the count can be reseted by sending a ResetEvent.
class CounterThread : public UThreadNode, public UEventsHandler
{
public:
	CounterThread() : state_(0), count_(0) {}
	virtual ~CounterThread() {this->join(true);}

protected:
	virtual void mainLoop() {
		if(state_ == 1) {
			state_ = 0;
			// Do a long initialization, reset memory or other special long works... here
			// we reset the count. This could be done in the handleEvent() but
			// with many objects, it is more safe to do it here (in the thread's loop). A safe
			// way could be also to use a UMutex to protect this initialization in
			// the handleEvent(), but it is not recommended to do long works in handleEvent()
			// because this will add latency in the UEventsManager dispatching events loop.
			count_ = 0; // Reset the count
			printf("Reset!\n");
		}

		// Do some works...
		printf("count=%d\n", count_++);
		uSleep(100); // wait 100 ms
	}
	virtual void handleEvent(UEvent * event) {
		if(event->getClassName().compare("ResetEventt") == 0) {
			state_ = 1;
		}
	}
private:
	int state_;
	int count_;
};

int main(int argc, char * argv[])
{
	CounterThread counter;
	counter.start();
	UEventsManager::addHandler(&counter);

	uSleep(500); // wait 500 ms before sending a reset event
	UEventsManager::post(new ResetEventt());
	uSleep(500); // wait 500 ms before termination

	UEventsManager::removeHandler(&counter);
	counter.join(true); // Kill and wait to finish
	return 0;
}
