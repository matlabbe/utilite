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

#ifndef _Mutex_Posix_
#define _Mutex_Posix_

#include <pthread.h>

class UMutex
{
  mutable pthread_mutex_t M;
  void operator=(UMutex &M) {}
  UMutex( const UMutex &M ) {}

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

  int Lock() const
  { return pthread_mutex_lock(&M); }

  int Lock_Try() const
  { return pthread_mutex_trylock(&M); }

  int Unlock() const
  { return pthread_mutex_unlock(&M); }
};

#endif // !_Mutex_Posix_
