/////////////////////////////////////////////////////////////////////
//  Written by Phillip Sitbon
//  Copyright 2003
//
//  Modified by Mathieu Labbe
//
//  Posix/Semaphore.h
//    - Resource counting mechanism
//
/////////////////////////////////////////////////////////////////////

#ifndef _Semaphore_Posix_
#define _Semaphore_Posix_

#include <pthread.h>

class USemaphore
{
public:
  USemaphore( int init = 0 )
  {
	  _available = init;
	  pthread_mutex_init(&_waitMutex, NULL);
	  pthread_cond_init(&_cond, NULL);
  }

  virtual ~USemaphore()
  {
	  pthread_cond_destroy(&_cond);
	  pthread_mutex_destroy(&_waitMutex);
  }

  void acquire(int n = 1)
  {
	  pthread_mutex_lock(&_waitMutex);
	  while (n > _available)
	  {
		  int code;
		  while(1)
		  {
		  	  code = pthread_cond_wait(&_cond, &_waitMutex);
		  	  //ULOGGER_DEBUG("code = %d", code);
		  	  //if(code == 0)
		  		//  continue;
		  	  break;
	  	  }
	  }
	  _available -= n;
	  pthread_mutex_unlock(&_waitMutex);
  }

  bool acquireTry(int n)
  {
	  pthread_mutex_lock(&_waitMutex);
	  if(n > _available)
	  {
		  pthread_mutex_unlock(&_waitMutex);
		  return false;
	  }
	  _available -= n;
	  pthread_mutex_unlock(&_waitMutex);
	  return true;
  }

  void release(int n = 1)
  {
	  pthread_mutex_lock(&_waitMutex);
	  _available += n;
	  pthread_cond_broadcast(&_cond);
	  pthread_mutex_unlock(&_waitMutex);
  }

  int value()
  {
	  int value = 0;
	  pthread_mutex_lock(&_waitMutex);
	  value = _available;
	  pthread_mutex_unlock(&_waitMutex);
	  return value;
  }

private:
  pthread_mutex_t _waitMutex;
  pthread_cond_t _cond;
  int _available;

};

#endif // !_Semaphore_Posix_
