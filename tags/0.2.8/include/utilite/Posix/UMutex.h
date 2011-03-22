/////////////////////////////////////////////////////////////////////
//  Written by Phillip Sitbon
//  Copyright 2003
//
//  Modified by Mathieu Labbe
//
//  Posix/Mutex.h
//    - Resource locking mechanism using Posix mutexes
//
/////////////////////////////////////////////////////////////////////

#ifndef _U_Mutex_Posix_
#define _U_Mutex_Posix_

#include <pthread.h>

class UMutex
{
public:

  UMutex()
  {
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&M,&attr);
    pthread_mutexattr_destroy(&attr);
  }

  virtual ~UMutex()
  { pthread_mutex_unlock(&M); pthread_mutex_destroy(&M); }

  int lock() const
  { return pthread_mutex_lock(&M); }

  int lockTry() const
  { return pthread_mutex_trylock(&M); }

  int unlock() const
  { return pthread_mutex_unlock(&M); }

private:
  mutable pthread_mutex_t M;
  void operator=(UMutex &M) {}
  UMutex( const UMutex &M ) {}
};

#endif // !_U_Mutex_Posix_
