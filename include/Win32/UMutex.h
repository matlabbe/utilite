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

#include "UWin32.h"

class UTILITE_EXP UMutex
{

public:
  UMutex()
  { InitializeCriticalSection(&C); }

  virtual ~UMutex()
  { DeleteCriticalSection(&C); }

  int lock() const
  { EnterCriticalSection(&C); return 0; }

#if(_WIN32_WINNT >= 0x0400)
  int lockTry() const
  { return (TryEnterCriticalSection(&C)?0:EBUSY); }
#endif

  int unlock() const
  { LeaveCriticalSection(&C); return 0; }

private:
  mutable CRITICAL_SECTION C;
  void operator=(UMutex &M) {}
  UMutex( const UMutex &M ) {}
};

#endif // !_Mutex_Win32_
