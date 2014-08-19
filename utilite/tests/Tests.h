/*
Copyright (c) 2008-2014, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TESTS_H
#define TESTS_H

#include <cppunit/TestFixture.h>
#include <cppunit/TestCaller.h>

#include <cppunit/extensions/HelperMacros.h>

#include "utilite/ULogger.h"
#include "utilite/UDirectory.h"

class Tests : public CppUnit::TestFixture  {

  CPPUNIT_TEST_SUITE( Tests );
  CPPUNIT_TEST( testThread );
  CPPUNIT_TEST( testThreadNodeMany );
  CPPUNIT_TEST( testSemaphore );
  CPPUNIT_TEST( testMutex );
  CPPUNIT_TEST( testEventsManager );
  //CPPUNIT_TEST( testConsoleLogger );
  CPPUNIT_TEST( testFileLogger );
  CPPUNIT_TEST( testConversion );
  CPPUNIT_TEST( testUtilStl );
  CPPUNIT_TEST( testDirectory );
  CPPUNIT_TEST( testFile );
  CPPUNIT_TEST( testMathFunctions );
  CPPUNIT_TEST( testTimer );
  CPPUNIT_TEST( testObjDeletionThread );
  CPPUNIT_TEST_SUITE_END();

private:
public:
  static const char* TEST_OUTPUT;
  static const char* LOG_FILE_NAME;
  static const char* FILE_LOGGER_TEST_FILE;

  void setUp()
  {
	  if(!UDir::exists("./LogTestUtil"))
	  {
		  UDir::makeDir("./LogTestUtil");
	  }

	  ULogger::reset();
	  ULogger::setType(ULogger::kTypeConsole);
	  ULogger::setLevel(ULogger::kError);
	  //ULogger::setLevel(ULogger::kDebug);
  }

  void tearDown() 
  {
	  ULogger::reset();
	  ULogger::setType(ULogger::kTypeConsole);
	  ULogger::setLevel(ULogger::kError);
	  //ULogger::setLevel(ULogger::kDebug);
  }

  void testThread();
  void testThreadNodeMany();
  void testSemaphore();
  void testMutex();
  void testEventsManager();
  void testConsoleLogger();
  void testFileLogger();
  void testConversion();
  void testUtilStl();
  void testDirectory();
  void testFile();
  void testMathFunctions();
  void testTimer();
  void testObjDeletionThread();
};

#endif
