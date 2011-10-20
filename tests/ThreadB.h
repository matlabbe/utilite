#ifndef THREADB_H
#define THREADB_H

#include "utilite/UEventsManager.h"
#include "EventA.h"
#include "EventB.h"
#include "utilite/UThreadNode.h"

#include "utilite/ULogger.h"


#include <string>

class ThreadB : public UEventsHandler, public UThreadNode
{
public:
    ThreadB(const int &workTime, const std::string &msg) : _workTime(workTime), _msg(msg){}
    ~ThreadB()
    {
    	UEventsManager::removeHandler(this);
    	join(this);
    }

    virtual void handleEvent(UEvent* anEvent)
    {
    	if(this->isRunning())
    	{
			if(anEvent->getClassName().compare("EventA") == 0)
			{
				doSomethingWithEvent((EventA*)anEvent);
				this->kill();
			}
    	}
    }
    
protected:
    virtual void mainLoop()
    {
        doSomeWork();
        UEventsManager::post(new EventB(EventB::TEST, _msg));
    }

private:
    void doSomeWork() const
    {
    	uSleep(_workTime);
    }

    void doSomethingWithEvent(EventA* anEvent) const
    {
        UINFO("ThreadB received a msg : \" %s \"", anEvent->getMsg().c_str());
    }

    int _workTime;
    std::string _msg;
};

#endif