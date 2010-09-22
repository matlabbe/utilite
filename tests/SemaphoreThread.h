/*
 * SemaphoreThread.h
 *
 *  Created on: Jun 11, 2010
 *      Author: MatLab
 */

#ifndef SEMAPHORETHREAD_H_
#define SEMAPHORETHREAD_H_

#include "UStateThread.h"

class SemaphoreThread : public UStateThread
{
public:
	SemaphoreThread(USemaphore * sem) : _sem(sem), _count(0) {}
    virtual ~SemaphoreThread() {this->killSafely();}
    int count() {return _count;}
protected:

private:
    virtual void killSafelyInner()
    {
    	if(_sem)
    	{
    		_sem->release();
    	}
    }

    virtual void threadInnerLoop()
    {
    	if(_sem)
    	{
    		_sem->acquire();
    		SLEEP(50);
    		_count++;
    	}
    }

private:
    USemaphore * _sem;
    int _count;
};

#endif /* SEMAPHORETHREAD_H_ */
