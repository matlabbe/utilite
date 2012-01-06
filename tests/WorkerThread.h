#ifndef WORKERTHREAD_H
#define WORKERTHREAD_H

#include "utilite/UThreadNode.h"

class WorkerThread : public UThreadNode
{
public:
	WorkerThread(){}
    ~WorkerThread()
    {
    	join(this);
    }
    
protected:
    virtual void mainLoop()
    {
        doSomeWork();
    }

private:
    void doSomeWork() const
    {
    	int a = 0;
    	a+=1;
    }
};

#endif
