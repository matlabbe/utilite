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
