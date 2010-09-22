#ifndef SIMPLESTATETHREAD_H
#define SIMPLESTATETHREAD_H

#include "UStateThread.h"

#include <string>

class SimpleStateThread : public UStateThread
{
public:
	SimpleStateThread(bool killItself = false) : _killItself(killItself) {}
    virtual ~SimpleStateThread() {this->killSafely();}

protected:

private:
    virtual void threadInnerLoop()
    {
    	if(_killItself)
    	{
    		killSafely();
    	}
    	else
    	{
    		doSomeWork();
    	}
    }

    void doSomeWork() const
    {
        SLEEP(10);
    }

private:
    bool _killItself;
};

#endif
