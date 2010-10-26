#ifndef TESTS_H
#define TESTS_H

#include <cppunit/TestFixture.h>
#include <cppunit/TestCaller.h>

#include <cppunit/extensions/HelperMacros.h>

#include "utilite/ULogger.h"
#include "utilite/UDirectory.h"

class Tests : public CppUnit::TestFixture  {

  CPPUNIT_TEST_SUITE( Tests );
  CPPUNIT_TEST( testStateThread );
  CPPUNIT_TEST( testSemaphore );
  CPPUNIT_TEST( testEventsManager );
  //CPPUNIT_TEST( testConsoleLogger );
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

  void setUp()
  {
	  if(!UDirectory::exists("./LogTestUtil"))
	  {
		  UDirectory::makeDir("./LogTestUtil");
	  }

	  ULogger::reset();
	  ULogger::setType(ULogger::kTypeConsole);
	  ULogger::setLevel(ULogger::kError);
	  //Util::Logger::setLevel(Util::Logger::kDebug);
  }

  void tearDown() 
  {
	  ULogger::reset();
	  ULogger::setType(ULogger::kTypeConsole);
	  ULogger::setLevel(ULogger::kError);
	  //Util::Logger::setLevel(Util::Logger::kDebug);
  }

  void testStateThread();
  void testSemaphore();
  void testEventsManager();
  void testConsoleLogger();
  void testConversion();
  void testUtilStl();
  void testDirectory();
  void testFile();
  void testMathFunctions();
  void testTimer();
  void testObjDeletionThread();
};

#endif
