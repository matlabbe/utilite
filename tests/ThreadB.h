#ifndef THREADB_H
#define THREADB_H

#include "UEventsManager.h"
#include "EventA.h"
#include "EventB.h"
#include "UStateThread.h"

#include "ULogger.h"


#include <string>

class ThreadB : public UEventsHandler, public UStateThread
{
public:
    ThreadB(const int &workTime, const std::string &msg) : _workTime(workTime), _msg(msg){}
    ~ThreadB()
    {
    	UEventsManager::removeHandler(this);
    	this->killSafely();
    }

    virtual void handleEvent(UEvent* anEvent)
    {
    	if(this->isRunning())
    	{
			if(anEvent->getClassName().compare("EventA") == 0)
			{
				doSomethingWithEvent((EventA*)anEvent);
				this->killSafely();
			}
    	}
    }
    
protected:
    virtual void threadInnerLoop()
    {
        doSomeWork();
        UEventsManager::postEvent(new EventB(EventB::TEST, _msg));
    }

private:
    void doSomeWork() const
    {
        SLEEP(_workTime);
    }

    void doSomethingWithEvent(EventA* anEvent) const
    {
        ULogger::write("ThreadB received a msg : \" %s \"", anEvent->getMsg().c_str());
    }

    int _workTime;
    std::string _msg;
};

#endif
