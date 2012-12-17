#ifndef SIMPLESTATETHREAD_H
#define SIMPLESTATETHREAD_H

#include "utilite/UThreadNode.h"

#include <string>

class SimpleStateThread : public UThreadNode
{
public:
	SimpleStateThread(bool killItself = false) : _killItself(killItself) {}
    virtual ~SimpleStateThread() {join(true);}

protected:

private:
    virtual void mainLoop()
    {
    	if(_killItself)
    	{
    		doSomeWork();
    		kill();
    	}
    	else
    	{
    		doSomeWork();
    	}
    }

    void doSomeWork() const
    {
    	//play with dynamic memory
    	float * array = new float[8820];
    	uSleep(10);
    	for(int i=0; i<8820; ++i)
    	{
    		array[i] = 0;
    	}
    	delete array;
    }

private:
    bool _killItself;
};

#endif
