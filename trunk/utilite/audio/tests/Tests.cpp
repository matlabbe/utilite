
#include <cppunit/config/SourcePrefix.h>
#include "Tests.h"

#include "utilite/UAudioCaptureFile.h"
#include "utilite/UAudioCaptureMic.h"
#include "utilite/UAudioPlayer.h"
#include "utilite/UAudioPlayerTone.h"
#include "utilite/UWav.h"
#include "utilite/UMp3Encoder.h"
#include "stdio.h"

#include <utilite/ULogger.h>
#include <utilite/UFile.h>

#define TEST_LENGTH_SEC 2 // seconds
#define TEST_SOUND_WAV "./TestSound.wav"
#define TEST_SOUND_STEREO_WAV "./TestSoundStereo.wav"
#define TEST_SOUND_MID "./TestSound.mid"
#ifdef BUILT_WITH_LAME
#define TEST_SOUND_MP3 "./TestSound.mp3"
#endif
#define TEST_RECORD_WAV "./testRecord.wav"
#define TEST_METRONOME_WAV "./Metronome.wav"
CPPUNIT_TEST_SUITE_REGISTRATION( Tests );

void Tests::setUp()
{
    //Logger::setType(Logger::TypeFile, "test.txt", false);
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
}

void Tests::tearDown()
{
}

void Tests::testMicRecorder()
{
    //
    // getRecordDrivers
    //
    std::vector<std::string> drivers = UAudioCaptureMic::getRecordDrivers();
    if(!drivers.size())
    {
    	UFATAL("Micro required for audio lib tests...");
    }
    for(unsigned int i=0; i<drivers.size(); i++)
    {
        UDEBUG("Record driver %d = %s", i, drivers[i].c_str());
    }
    CPPUNIT_ASSERT(drivers.size() > 0);

    //
    // recording to a wav
    //
    std::string fileName = TEST_SOUND_WAV;//TEST_RECORD_WAV;
    if(UFile::exists(fileName))
    {
        CPPUNIT_ASSERT(remove(fileName.c_str()) == 0);
    }

    UAudioCapture* recorder = new UAudioCaptureMic(fileName, 100, 0);
    recorder->init();
    recorder->start();
    std::vector<char> frame;
    int frameId = 0;
    int maxFrame = (44100*TEST_LENGTH_SEC)/1024;
    while(recorder->getNextFrame(frame, frameId) && frameId < maxFrame)
    { 
    }

    recorder->stop();

    CPPUNIT_ASSERT(frameId == maxFrame);

    recorder->close(); // This will finalize the audio file

    CPPUNIT_ASSERT(UFile::exists(fileName));

    delete recorder;
}

void Tests::testFileRecorder()
{
    std::string fileName = TEST_SOUND_WAV;
    CPPUNIT_ASSERT(UFile::exists(fileName));
    UAudioCapture* recorder = new UAudioCaptureFile(fileName);
    recorder->init();
    recorder->start();
    std::vector<char> frame;
    unsigned int frameId = 0;

    while(recorder->getNextFrame(frame, true))
    {
        frameId++;
        //UINFO("%d : l=%d \n", frameNumber, frame.getLength());
        /*rtabmap::Frame data = frame;
        std::ostringstream oss;
        for(int i=0; i<frame.size(); i++)
        {
            oss << (short)(data[i]*32768) << '    ';
        }
        UINFO(oss.str().c_str());*/
        if(frameId == 40)
        {
        	recorder->stop();
        	recorder->start();
        }
    }

    recorder->stop();

    char buf[30] = {0};
    sprintf(buf, "FrameId = %d", frameId);
    CPPUNIT_ASSERT_MESSAGE(buf, frameId == 87);

    delete recorder;
}

void Tests::testFilePlayRecorder()
{
    std::string fileName = TEST_SOUND_WAV;
    CPPUNIT_ASSERT(UFile::exists(fileName));
   UAudioCapture* recorder = new UAudioCaptureFile(fileName, true, 1024);
    recorder->init();
    recorder->start();
    std::vector<char> frame;
    int frameId = 0;

    while(recorder->getNextFrame(frame, frameId))
    {
        //UINFO("%d : l=%d \n", frameNumber, frame.getLength());
        /*rtabmap::Frame data = frame;
        std::ostringstream oss;
        for(int i=0; i<frame.size(); i++)
        {
            oss << (short)(data[i]*32768) << '    ';
        }
        UINFO(oss.str().c_str());*/
    	if(frameId == 40)
		{
			recorder->stop();
			recorder->start();
		}
    }

    recorder->stop();

    char buf[30] = {0};
    sprintf(buf, "FrameId = %d", frameId);
    CPPUNIT_ASSERT_MESSAGE(buf, frameId == 87);

    delete recorder;
}

void Tests::testFileRecorderStereo()
{
    std::string fileName = TEST_SOUND_STEREO_WAV;
    CPPUNIT_ASSERT(UFile::exists(fileName));
   UAudioCapture* recorder = new UAudioCaptureFile(fileName);
    recorder->init();
    recorder->start();
    std::vector<char> frame;
    unsigned int frameId = 0;

    while(recorder->getNextFrame(frame, frameId))
    {
        //UINFO("%d : l=%d \n", frameNumber, frame.getLength());
    	std::vector<char> data = frame;
        std::ostringstream oss;
        for(unsigned int i=0; i<frame.size(); i++)
        {
            oss << (short)(data[i]*32768) << ' ';
        }
        UINFO(oss.str().c_str());
    }

    recorder->stop();

    char buf[30] = {0};
    sprintf(buf, "FrameId = %d", frameId);
    CPPUNIT_ASSERT_MESSAGE(buf, frameId == 72);

    delete recorder;
}

void Tests::testPlayRecorderStereo()
{
    std::string fileName = TEST_SOUND_STEREO_WAV;
    CPPUNIT_ASSERT(UFile::exists(fileName));
   UAudioCapture* recorder = new UAudioCaptureFile(fileName, 1024, true);
    recorder->init();
    recorder->start();
    std::vector<char> frame;
    unsigned int frameId = 0;

    while(recorder->getNextFrame(frame, frameId))
    {
        //UINFO("%d : l=%d \n", frameNumber, frame.getLength());
    	std::vector<char> data = frame;
        std::ostringstream oss;
        for(unsigned int i=0; i<frame.size(); i++)
        {
            oss << (short)(data[i]*32768) << ' ';
        }
        UINFO(oss.str().c_str());
    }

    recorder->stop();

    char buf[30] = {0};
    sprintf(buf, "FrameId = %d", frameId);
    CPPUNIT_ASSERT_MESSAGE(buf, frameId == 72);

    delete recorder;
}

void Tests::testTonePlayer()
{
    bool ok = true;
   UAudioPlayer* player = new UAudioPlayerTone(440.0f, 0.4f, 200, 300);
    player->play();
    uSleep(TEST_LENGTH_SEC*1000/4);
    player->stop();
    uSleep(TEST_LENGTH_SEC*1000/4);
    player->play();
    uSleep(TEST_LENGTH_SEC*1000/4);
    player->stop();

    delete player;

    CPPUNIT_ASSERT(ok);
}

void Tests::testTonePlayerWav()
{
    bool ok = true;
   UAudioPlayer* player = new UAudioPlayerTone(TEST_METRONOME_WAV, 0.4f, 200);
    player->play();
    uSleep(TEST_LENGTH_SEC*1000/4);
    player->stop();
    uSleep(TEST_LENGTH_SEC*1000/4);
    player->play();
    uSleep(TEST_LENGTH_SEC*1000/4);
    player->stop();

    delete player;

    CPPUNIT_ASSERT(ok);
}

void Tests::testFilePlayerWav()
{
    // Play a wav
    const char* fileName = TEST_SOUND_WAV;
    CPPUNIT_ASSERT(UFile::exists(fileName));
    
    bool ok = true;
   UAudioPlayer* player = new UAudioPlayer(fileName);
    CPPUNIT_ASSERT(!player->init());

    unsigned int soundLength = player->getSoundLengthMs();
    char buf[30] = {0};
    sprintf(buf, "soundLength = %d", soundLength);
    CPPUNIT_ASSERT_MESSAGE(buf, soundLength >= 2020 && soundLength <= 2040);

    player->play();
    /*uSleep(500);
    player->stop();
    //Sleep(100);
    player->init();
    player->setPositionMs(500);
    player->play();*/
    uSleep(soundLength);
    player->stop();

    delete player;

    CPPUNIT_ASSERT(ok);
}

void Tests::testFilePlayerMid()
{
    // Play a midi
    const char* fileName = TEST_SOUND_MID;
    CPPUNIT_ASSERT(UFile::exists(fileName));
    
    bool ok = true;
   UAudioPlayer* player = new UAudioPlayer(fileName);
    CPPUNIT_ASSERT(!player->init());

    unsigned int soundLength = player->getSoundLengthMs();
    char buf[30] = {0};
    sprintf(buf, "soundLength = %d", soundLength);
    CPPUNIT_ASSERT_MESSAGE(buf, soundLength == 60010);

    player->play();
    player->setPositionMs(2000);
    uSleep(2100);
    player->stop();

    delete player;
    CPPUNIT_ASSERT(ok);
}

void Tests::testWav()
{
    std::string fileName = TEST_SOUND_WAV;
    CPPUNIT_ASSERT(UFile::exists(fileName));

   UWav wav;
    int ret = wav.init(fileName.c_str());
    CPPUNIT_ASSERT(ret == 0);
}

#ifdef BUILT_WITH_LAME
void Tests::testFilePlayerMp3()
{
    // Play a mp3
    const char* fileName = TEST_SOUND_MP3;
    CPPUNIT_ASSERT(UFile::exists(fileName));
    
    bool ok = true;
   UAudioPlayer* player = new UAudioPlayer(fileName);
    CPPUNIT_ASSERT(!player->init());

    unsigned int soundLength = player->getSoundLengthMs();
    char buf[30] = {0};
    sprintf(buf, "soundLength = %d", soundLength);
    CPPUNIT_ASSERT_MESSAGE(buf, soundLength == 2142);

    player->init();
    player->play();
/*
    player->setPositionMs(500);
    player->play();*/
    uSleep(soundLength);
    player->stop();

    delete player;

    CPPUNIT_ASSERT(ok);
}

void Tests::testMp3Encoder()
{
    char buf[30] = {0};
    long fileLength = 0;
    std::string fileName;
    std::string fileOut;
   UMp3Encoder mp3Encoder;

    // Mono
    fileName = TEST_SOUND_WAV;
    CPPUNIT_ASSERT_MESSAGE(TEST_SOUND_WAV, UFile::exists(fileName));

    fileOut = TEST_SOUND_MP3;
    //fileOut = fileName;
    //fileOut.append(".mp3");
    CPPUNIT_ASSERT(mp3Encoder.encode(fileName, fileOut) == 0);

    fileLength = UFile::length(fileOut);
    sprintf(buf, "Mono file length = %ld", fileLength);
    CPPUNIT_ASSERT_MESSAGE(buf, UFile::length(fileOut) == 16717);

    //Stereo
    // Test a stereo file
    /*fileName = TEST_SOUND_STEREO_WAV;
    CPPUNIT_ASSERT_MESSAGE(TEST_SOUND_STEREO_WAV, UFile::exists(fileName));

    fileOut = fileName;
    fileOut.append(".mp3");
    CPPUNIT_ASSERT(mp3Encoder.encode(fileName, fileOut) == 0);

    fileLength = UFile::length(fileOut);
    sprintf(buf, "Stereo file length = %ld", fileLength);
    CPPUNIT_ASSERT_MESSAGE(buf, UFile::length(fileOut) == 28002);*/
}
#endif
