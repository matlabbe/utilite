#ifndef SIMPLESTATETHREAD_H
#define SIMPLESTATETHREAD_H

#include "UThreadNode.h"

#include <string>

class SimpleStateThread : public UThreadNode
{
public:
	SimpleStateThread(bool killItself = false) : _killItself(killItself) {}
    virtual ~SimpleStateThread() {this->kill();}

protected:

private:
    virtual void mainLoop()
    {
    	if(_killItself)
    	{
    		kill();
    	}
    	else
    	{
    		doSomeWork();
    	}
    }

    void doSomeWork() const
    {
    	uSleep(10);
    }

private:
    bool _killItself;
};

#endif
