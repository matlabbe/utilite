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
#ifndef _U_Semaphore_Win32_
#define _U_Semaphore_Win32_

#include "utilite/Win32/UWin32.h"

#define SEM_VALUE_MAX ((int) ((~0u) >> 1))

class UTILITE_EXP USemaphore
{
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

  int acquireTry() const
  { return ((WaitForSingleObject((HANDLE)S,INFINITE)==WAIT_OBJECT_0)?0:EAGAIN); }

  int release(int n = 1) const
  { return (ReleaseSemaphore((HANDLE)S,n,0)?0:ERANGE); }

  int value() const
  { LONG V = -1; ReleaseSemaphore((HANDLE)S,0,&V); return V; }

  void reset( int init = 0 )
  {
    CloseHandle(S);
    S = CreateSemaphore(0,init,SEM_VALUE_MAX,0);
  }

private:
  HANDLE S;
  void operator=(const USemaphore &S){}
  USemaphore(const USemaphore &S){}
};

#endif // !_U_Semaphore_Win32_
