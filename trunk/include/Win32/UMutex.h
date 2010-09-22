/////////////////////////////////////////////////////////////////////
//  Written by Phillip Sitbon
//  Copyright 2003
//
//  Modified by Mathieu Labbe
//
//  Win32/Mutex.h
//    - Resource locking mechanism using Critical Sections
//
/////////////////////////////////////////////////////////////////////

#ifndef _Mutex_Win32_
#define _Mutex_Win32_

#include "Win32.h"

class UTILITE_EXP UMutex
{
  mutable CRITICAL_SECTION C;
  void operator=(UMutex &M) {}
  UMutex( const UMutex &M ) {}

  public:

  UMutex()
  { InitializeCriticalSection(&C); }

  virtual ~UMutex()
  { DeleteCriticalSection(&C); }

  int Lock() const
  { EnterCriticalSection(&C); return 0; }

#if(_WIN32_WINNT >= 0x0400)
  int Lock_Try() const
  { return (TryEnterCriticalSection(&C)?0:EBUSY); }
#endif

  int Unlock() const
  { LeaveCriticalSection(&C); return 0; }
};

#endif // !_Mutex_Win32_
