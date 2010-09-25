/*
 * SemaphoreThread.h
 *
 *  Created on: Jun 11, 2010
 *      Author: MatLab
 */

#ifndef SEMAPHORETHREAD_H_
#define SEMAPHORETHREAD_H_

#include "UThreadNode.h"

class SemaphoreThread : public UThreadNode
{
public:
	SemaphoreThread(USemaphore * sem) : _sem(sem), _count(0) {}
    virtual ~SemaphoreThread() {this->kill();}
    int count() {return _count;}
protected:

private:
    virtual void killCleanup()
    {
    	if(_sem)
    	{
    		_sem->release();
    	}
    }

    virtual void mainLoop()
    {
    	if(_sem)
    	{
    		_sem->acquire();
    		uSleep(50);
    		_count++;
    	}
    }

private:
    USemaphore * _sem;
    int _count;
};

#endif /* SEMAPHORETHREAD_H_ */
