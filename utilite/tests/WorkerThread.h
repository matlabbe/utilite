#ifndef WORKERTHREAD_H
#define WORKERTHREAD_H

#include "utilite/UThread.h"

class WorkerThread : public UThread
{
public:
	WorkerThread(){}
    ~WorkerThread()
    {
    	join(true);
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
