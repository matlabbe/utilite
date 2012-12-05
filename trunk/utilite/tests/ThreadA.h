#ifndef THREADA_H
#define THREADA_H

#include "utilite/UEventsManager.h"
#include "EventA.h"
#include "EventB.h"
#include "utilite/UThreadNode.h"

#include "utilite/ULogger.h"


#include <string>

class ThreadA : public UEventsHandler, public UThreadNode
{
public:
    ThreadA(const int &workTime, const std::string &msg) : _workTime(workTime), _msg(msg){}
    virtual ~ThreadA()
    {
    	UEventsManager::removeHandler(this);
    	join(true);
    }

    virtual void handleEvent(UEvent* anEvent)
    {
    	if(this->isRunning())
    	{
			if(anEvent->getClassName().compare("EventB") == 0)
			{
				doSomethingWithEvent((EventB*)anEvent);
				kill();
			}
    	}
    }

protected:

private:
    virtual void mainLoop()
    {
        doSomeWork();
        UEventsManager::post(new EventA(EventA::TEST, _msg));
    }

    void doSomeWork() const
    {
        uSleep(_workTime);
    }

    void doSomethingWithEvent(EventB* anEvent) const
    {
        UINFO("ThreadA received a msg : \" %s \"", anEvent->getMsg().c_str());
    }

    int _workTime;
    std::string _msg;
};

#endif
