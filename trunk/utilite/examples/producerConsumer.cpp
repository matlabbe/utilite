#include <utilite/UThread.h>
#include <utilite/UEventsHandler.h>
#include <utilite/ULogger.h>
#include <utilite/UEvent.h>
#include <utilite/UEventsManager.h>

/**
 * Example of the producer-consumer problem.
 */

// A simple event
class SimpleEvent : public UEvent
{
public:
	SimpleEvent(int level) :
		UEvent(level) {}
	virtual ~SimpleEvent() {}
	virtual std::string getClassName() const {return "SimpleEvent";}
};

// The Producer class
class Producer : public UThread
{
public:
	Producer(const std::string & name, int ms) :
		name_(name),
		rate_(ms) ,
		count_(0) {}

	~Producer() {
		this->join(true);
	}

protected:
	virtual void mainLoop() {
		uSleep(rate_);
		UINFO("%s posting %d", name_.c_str(), count_);
		UEventsManager::post(new SimpleEvent(count_++));
	}

private:
	const std::string name_;
	int rate_;
	int count_;
};

// The Consumer class
class Consumer : public UEventsHandler
{
public:
	Consumer() {}
	~Consumer() {}

protected:
	virtual void handleEvent(UEvent * event) {
		if(event->getClassName().compare("SimpleEvent"))
		{
			UINFO("Consumer received %d", event->getCode());
		}
	}
};

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setPrintWhere(false);

	Consumer c;
	UEventsManager::addHandler(&c);

	Producer p500("p500", 500); //post each 500 ms
	Producer p1000("p1000", 1000); //post each 1000 ms

	// Start the producer threads
	p500.start();
	p1000.start();

	uSleep(3500);
	return 0;
}

