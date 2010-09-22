#ifndef THREADA_H
#define THREADA_H

#include "UEventsManager.h"
#include "EventA.h"
#include "EventB.h"
#include "UStateThread.h"

#include "ULogger.h"


#include <string>

class ThreadA : public UEventsHandler, public UStateThread
{
public:
    ThreadA(const int &workTime, const std::string &msg) : _workTime(workTime), _msg(msg){}
    virtual ~ThreadA()
    {
    	UEventsManager::removeHandler(this);
    	this->killSafely();
    }

    virtual void handleEvent(UEvent* anEvent)
    {
    	if(this->isRunning())
    	{
			if(anEvent->getClassName().compare("EventB") == 0)
			{
				doSomethingWithEvent((EventB*)anEvent);
				this->killSafely();
			}
    	}
    }

protected:

private:
    virtual void threadInnerLoop()
    {
        doSomeWork();
        UEventsManager::postEvent(new EventA(EventA::TEST, _msg));
    }

    void doSomeWork() const
    {
        SLEEP(_workTime);
    }

    void doSomethingWithEvent(EventB* anEvent) const
    {
        ULogger::write("ThreadA received a msg : \" %s \"", anEvent->getMsg().c_str());
    }

    int _workTime;
    std::string _msg;
};

#endif
