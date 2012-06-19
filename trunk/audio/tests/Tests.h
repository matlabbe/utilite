
#include <cppunit/TestFixture.h>
#include <cppunit/TestCaller.h>

#include <cppunit/extensions/HelperMacros.h>

class Tests : public CppUnit::TestFixture  {

  CPPUNIT_TEST_SUITE( Tests );
  CPPUNIT_TEST( testMicRecorder );
  CPPUNIT_TEST( testFileRecorder );
  CPPUNIT_TEST( testFilePlayRecorder );
  //CPPUNIT_TEST( testFileRecorderStereo );
  //CPPUNIT_TEST( testPlayRecorderStereo );
  CPPUNIT_TEST( testWav );
#ifdef BUILT_WITH_LAME
  CPPUNIT_TEST( testMp3Encoder );
  CPPUNIT_TEST( testFilePlayerMp3 );
#endif
  CPPUNIT_TEST( testTonePlayer );
  //CPPUNIT_TEST( testTonePlayerWav );
  CPPUNIT_TEST( testFilePlayerWav );
  //CPPUNIT_TEST( testFilePlayerMid );
  CPPUNIT_TEST_SUITE_END();

private:
public:
  void setUp();
  void tearDown();

  // UAudioRecorder tests
  void testMicRecorder();
  void testFileRecorder();
  void testFilePlayRecorder();
  void testFileRecorderStereo();
  void testPlayRecorderStereo();

  // UAudioPlayer tests
  void testFilePlayerWav();
  void testFilePlayerMid();
  void testTonePlayer();
  void testTonePlayerWav();

  //Others
  void testWav();
#ifdef BUILT_WITH_LAME
  void testFilePlayerMp3();
  void testMp3Encoder();
#endif
};
