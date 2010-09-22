/////////////////////////////////////////////////////////////////////
//  Written by Phillip Sitbon
//  Copyright 2003
//
//  Modified by Mathieu Labbe
//
//  Win32/Semaphore.h
//    - Resource counting mechanism
//
/////////////////////////////////////////////////////////////////////
#ifndef _Semaphore_Win32_
#define _Semaphore_Win32_

#include "Win32.h"

#define SEM_VALUE_MAX ((int) ((~0u) >> 1))

class UTILITE_EXP USemaphore
{
  HANDLE S;
  void operator=(const USemaphore &S){}
  USemaphore(const USemaphore &S){}

  public:
  USemaphore( int init = 0 )
  { S = CreateSemaphore(0,init,SEM_VALUE_MAX,0); }

  virtual ~USemaphore()
  { CloseHandle(S); }

  void acquire(int n = 1) const
  {
	  while(n-- > 0)
		  WaitForSingleObject((HANDLE)S,INFINITE);
  }

  int Wait_Try() const
  { return ((WaitForSingleObject((HANDLE)S,INFINITE)==WAIT_OBJECT_0)?0:EAGAIN); }

  int release(int n = 1) const
  { return (ReleaseSemaphore((HANDLE)S,n,0)?0:ERANGE); }

  int Value() const
  { LONG V = -1; ReleaseSemaphore((HANDLE)S,0,&V); return V; }

  void Reset( int init = 0 )
  {
    CloseHandle(S);
    S = CreateSemaphore(0,init,SEM_VALUE_MAX,0);
  }
};

#endif // !_Semaphore_Win32_
