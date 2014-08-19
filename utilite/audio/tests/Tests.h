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
