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

#include <cppunit/config/SourcePrefix.h>
#include "Tests.h"

//Headers for the test BEGIN
#include "utilite/UEventsManager.h"
#include "utilite/ULogger.h"
#include "ThreadA.h"
#include "ThreadB.h"
#include "WorkerThread.h"
#include <fstream>
#include <string.h>
#include "utilite/UConversion.h"
#include "utilite/UStl.h"
#include "SimpleStateThread.h"
#include "utilite/UFile.h"
#include "utilite/UDirectory.h"
#include "utilite/UMath.h"
#include "utilite/UTimer.h"
#include "utilite/UObjDeletionThread.h"
#include "SemaphoreThread.h"

//Headers for the test END

const char* Tests::TEST_OUTPUT = "[ INFO] Program started...[ INFO] ThreadB received a msg : \" Sweet !!! what is wrote on my back ? \"[ INFO] ThreadA received a msg : \" Dude !!! what is wrote on my back ? \"[ INFO] Killing threads...";
const char* Tests::LOG_FILE_NAME = "./TestUtilitiesLib.txt";
const char* Tests::FILE_LOGGER_TEST_FILE = "./TestFileLogger.txt";

CPPUNIT_TEST_SUITE_REGISTRATION( Tests );

static const char HEX2ASCII[256][2] =
{
	{'0','0'},{'0','1'},{'0','2'},{'0','3'},{'0','4'},{'0','5'},{'0','6'},{'0','7'},{'0','8'},{'0','9'},{'0','A'},{'0','B'},{'0','C'},{'0','D'},{'0','E'},{'0','F'},
	{'1','0'},{'1','1'},{'1','2'},{'1','3'},{'1','4'},{'1','5'},{'1','6'},{'1','7'},{'1','8'},{'1','9'},{'1','A'},{'1','B'},{'1','C'},{'1','D'},{'1','E'},{'1','F'},
	{'2','0'},{'2','1'},{'2','2'},{'2','3'},{'2','4'},{'2','5'},{'2','6'},{'2','7'},{'2','8'},{'2','9'},{'2','A'},{'2','B'},{'2','C'},{'2','D'},{'2','E'},{'2','F'},
	{'3','0'},{'3','1'},{'3','2'},{'3','3'},{'3','4'},{'3','5'},{'3','6'},{'3','7'},{'3','8'},{'3','9'},{'3','A'},{'3','B'},{'3','C'},{'3','D'},{'3','E'},{'3','F'},
	{'4','0'},{'4','1'},{'4','2'},{'4','3'},{'4','4'},{'4','5'},{'4','6'},{'4','7'},{'4','8'},{'4','9'},{'4','A'},{'4','B'},{'4','C'},{'4','D'},{'4','E'},{'4','F'},
	{'5','0'},{'5','1'},{'5','2'},{'5','3'},{'5','4'},{'5','5'},{'5','6'},{'5','7'},{'5','8'},{'5','9'},{'5','A'},{'5','B'},{'5','C'},{'5','D'},{'5','E'},{'5','F'},
	{'6','0'},{'6','1'},{'6','2'},{'6','3'},{'6','4'},{'6','5'},{'6','6'},{'6','7'},{'6','8'},{'6','9'},{'6','A'},{'6','B'},{'6','C'},{'6','D'},{'6','E'},{'6','F'},
	{'7','0'},{'7','1'},{'7','2'},{'7','3'},{'7','4'},{'7','5'},{'7','6'},{'7','7'},{'7','8'},{'7','9'},{'7','A'},{'7','B'},{'7','C'},{'7','D'},{'7','E'},{'7','F'},
	{'8','0'},{'8','1'},{'8','2'},{'8','3'},{'8','4'},{'8','5'},{'8','6'},{'8','7'},{'8','8'},{'8','9'},{'8','A'},{'8','B'},{'8','C'},{'8','D'},{'8','E'},{'8','F'},
	{'9','0'},{'9','1'},{'9','2'},{'9','3'},{'9','4'},{'9','5'},{'9','6'},{'9','7'},{'9','8'},{'9','9'},{'9','A'},{'9','B'},{'9','C'},{'9','D'},{'9','E'},{'9','F'},
	{'A','0'},{'A','1'},{'A','2'},{'A','3'},{'A','4'},{'A','5'},{'A','6'},{'A','7'},{'A','8'},{'A','9'},{'A','A'},{'A','B'},{'A','C'},{'A','D'},{'A','E'},{'A','F'},
	{'B','0'},{'B','1'},{'B','2'},{'B','3'},{'B','4'},{'B','5'},{'B','6'},{'B','7'},{'B','8'},{'B','9'},{'B','A'},{'B','B'},{'B','C'},{'B','D'},{'B','E'},{'B','F'},
	{'C','0'},{'C','1'},{'C','2'},{'C','3'},{'C','4'},{'C','5'},{'C','6'},{'C','7'},{'C','8'},{'C','9'},{'C','A'},{'C','B'},{'C','C'},{'C','D'},{'C','E'},{'C','F'},
	{'D','0'},{'D','1'},{'D','2'},{'D','3'},{'D','4'},{'D','5'},{'D','6'},{'D','7'},{'D','8'},{'D','9'},{'D','A'},{'D','B'},{'D','C'},{'D','D'},{'D','E'},{'D','F'},
	{'E','0'},{'E','1'},{'E','2'},{'E','3'},{'E','4'},{'E','5'},{'E','6'},{'E','7'},{'E','8'},{'E','9'},{'E','A'},{'E','B'},{'E','C'},{'E','D'},{'E','E'},{'E','F'},
	{'F','0'},{'F','1'},{'F','2'},{'F','3'},{'F','4'},{'F','5'},{'F','6'},{'F','7'},{'F','8'},{'F','9'},{'F','A'},{'F','B'},{'F','C'},{'F','D'},{'F','E'},{'F','F'}
};

void loop()
{

}

static SimpleStateThread threadStatic;

void Tests::testThread()
{
	//Logger::setType(Logger::kTypeConsole);
	//Logger::setLevel(Logger::kDebug);

	//ULOGGER_DEBUG("***threadStatic.start();***");
	CPPUNIT_ASSERT( threadStatic.isIdle() );
	threadStatic.start();
	CPPUNIT_ASSERT( threadStatic.isRunning() );
	threadStatic.kill();
	CPPUNIT_ASSERT( threadStatic.isKilled() );
	threadStatic.start();
	CPPUNIT_ASSERT( threadStatic.isRunning() );

	//ULOGGER_DEBUG("***SimpleStateThread thread;***");
	SimpleStateThread thread;
	CPPUNIT_ASSERT( thread.isIdle() );
	thread.start();
	CPPUNIT_ASSERT( thread.isRunning() );
	thread.kill();
	CPPUNIT_ASSERT( thread.isKilled() );

	SimpleStateThread threadAutoKill(true);
	CPPUNIT_ASSERT( threadAutoKill.isIdle() );
	threadAutoKill.start();
	uSleep(20);
	CPPUNIT_ASSERT( threadAutoKill.isIdle() );
	threadAutoKill.kill();
	CPPUNIT_ASSERT( threadAutoKill.isIdle() );

	//ULOGGER_DEBUG("***threadA->start();***");
	ThreadA  * threadA = new ThreadA(150, "Sweet !!! what is wrote on my back ?" );
	CPPUNIT_ASSERT( threadA->isIdle() );
	threadA->start();
	CPPUNIT_ASSERT( threadA->isRunning() );
	threadA->kill();
	CPPUNIT_ASSERT( threadA->isKilled() );
	delete threadA;

	//Test join
	SimpleStateThread threadToJoin(true);
	threadToJoin.start();
	threadToJoin.join();
	CPPUNIT_ASSERT( threadToJoin.isIdle() );

	//Test killing same thread with two other threads
	threadA = new ThreadA(1000, "Long working..." );
	UEventsManager::addHandler(threadA);
	threadA->start();
	uSleep(10);
	UEventsManager::post(new EventB(EventB::TEST, "Kill!")); // when receiving an EventB, thread A kills itself
	uSleep(1);
	delete threadA;


	//Test Affinity
	/*WorkerThread worker1;
	WorkerThread worker2;
	WorkerThread worker3;
	WorkerThread worker4;
	worker1.setAffinity(1);
	worker2.setAffinity(1);
	worker3.setAffinity(1);
	worker4.setAffinity(1);
	worker1.start();
	worker2.start();
	worker3.start();
	worker4.start();
	UERROR("");
	uSleep(10000);
	UThreadNode::join(&worker1);
	UThreadNode::join(&worker2);
	UThreadNode::join(&worker3);
	UThreadNode::join(&worker4);
	worker1.setAffinity(1);
	worker2.setAffinity(2);
	worker3.setAffinity(3);
	worker4.setAffinity(4);
	worker1.start();
	worker2.start();
	worker3.start();
	worker4.start();
	UERROR("");
	uSleep(10000);
	UThreadNode::join(&worker1);
	UThreadNode::join(&worker2);
	UThreadNode::join(&worker3);
	UThreadNode::join(&worker4);*/
}

void Tests::testThreadNodeMany()
{
	std::vector<ThreadA*> nodes(100);
	for(unsigned int i=0; i<nodes.size(); ++i)
	{
		nodes[i] = new ThreadA(i*10+10, uNumber2Str(i));
		UEventsManager::addHandler(nodes[i]);
		nodes[i]->start();
	}

	ThreadB threadB(2000, "B");
	threadB.start();
	for(int i=0; i<100; ++i)
	{
		SimpleStateThread t1(true);
		SimpleStateThread t2(true);
		SimpleStateThread t3(true);
		SimpleStateThread t4(true);
		SimpleStateThread t5(true);
		t1.start();
		t2.start();
		t3.start();
		t4.start();
		t5.start();
		uSleep(21);
	}
	threadB.join(true);

	for(unsigned int i=0; i<nodes.size(); ++i)
	{
		nodes[i]->join();
		delete nodes[i];
	}
}

void Tests::testSemaphore()
{
	//Logger::setType(Logger::kTypeConsole);
	//Logger::setLevel(Logger::kDebug);
	USemaphore sem;
	SemaphoreThread semThr(&sem);
	semThr.start();
	uSleep(100);
	sem.release();
	uSleep(100);
	sem.release(2);
	uSleep(60);
	sem.release(2);
	uSleep(240);
	sem.release(10);
	uSleep(510);
	sem.release(0);
	sem.release(1);
	uSleep(100);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(60);
	sem.release();
	uSleep(200);
	//printf("\n count = semThr.count()=%d", semThr.count());
	CPPUNIT_ASSERT( semThr.count() == 25);

	semThr.kill();
}

class MutexThread : public UThread
{
public:
	MutexThread(UMutex & mutex) :
		mutex_(mutex)
	{}
protected:
	virtual void mainLoop()
	{
		UScopeMutex sm(mutex_);
		uSleep(100);
		this->kill();
	}
private:
	UMutex & mutex_;
};

void Tests::testMutex()
{
	UMutex mutex;
	MutexThread t(mutex);
	t.start();
	while(t.isCreating())
	{
		// wait thread to be started
		uSleep(0);
	}
	UTimer time;
	UScopeMutex scopeMutex(mutex);
	CPPUNIT_ASSERT(time.elapsed() > 0.09); // at least 100 ms
	mutex.lock(); // should work, a mutex is recursive
	mutex.unlock();

}

void Tests::testEventsManager()
{
    /* Set logger type */
    //Logger::setType(Logger::kTypeConsole);
    ULogger::setType(ULogger::kTypeFile, LOG_FILE_NAME, false);
    ULogger::setPrintTime(false);
    ULogger::setPrintEndline(false);
    ULogger::setPrintWhere(false);
    ULogger::setLevel(ULogger::kInfo); // Disable log info

    UINFO("Program started...");

    /* Create tasks */

    ThreadA threadA(50, "Sweet !!! what is wrote on my back ?" );
    ThreadB threadB(100, "Dude !!! what is wrote on my back ?");

    /* Add handlers to the EventsManager */
    UEventsManager::addHandler(&threadA);
    UEventsManager::addHandler(&threadB);

    /* Start thread's task */
    threadA.start();
    threadB.start();

    uSleep(250); // wait 0.25 sec and exit

    /* Remove handlers */
    UEventsManager::removeHandler(&threadA);
    UEventsManager::removeHandler(&threadB);

    UINFO("Killing threads...");
    /* Kill threads      														*/
    /* -I recommend the use of kill(), the thread will finish normally.   */
    /*     All mutex locked by this thread will be unlocked. 					*/
    /* -Use kill() when we don't care about when the thread finishes...         */
    /*     The problem with this is if the the thread is calling a function     */
    /*     with a Mutex and it locks it, he will never unlock the mutex...      */
    /*     it's very bad because other threads can be waiting for this mutex    */
    /*     and they will never end.												*/
    threadA.kill();
    threadB.kill();

    // Setup variables of the test
    std::string resultStr;

    ULogger::reset();// Make sure the log file is closed

    // open the result output file
    std::fstream resultFile(LOG_FILE_NAME, std::fstream::in);

    CPPUNIT_ASSERT(resultFile.is_open());

	std::string line;
	while(std::getline(resultFile,line))
	{
		resultStr += line;
	}
	resultFile.close();
    
    std::string msg;
    msg += "***The test***\n";
    msg += TEST_OUTPUT;
    msg += "\n***The result***\n" + resultStr;

    CPPUNIT_ASSERT_MESSAGE(msg, strcmp(resultStr.c_str(), TEST_OUTPUT) == 0 );
}

void Tests::testConsoleLogger()
{
	/* Set logger type */
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setPrintTime(true);
	ULogger::setPrintEndline(true);
	//Logger::addLevel(-1); // Disable log with level

	ULogger::write(" << Hello world! >> ");
	ULogger::write(" << Hello world2! >> ");
	ULogger::write("%s", " << Hello world3! >> ");
}

void Tests::testFileLogger()
{
	/* Set logger type */
	ULogger::setBuffered(false);
	ULogger::setType(ULogger::kTypeFile, FILE_LOGGER_TEST_FILE, false);
	ULogger::setLevel(ULogger::kInfo);
	UINFO("start");
	ULogger::setBuffered(true);
	std::string longStr(1026, '#'); // size over fixed default size of 1024
	longStr.append(std::string("%f %d, %s"));
	UINFO(longStr.c_str(), 0.12345, 42, "test");
	UINFO("Small string without arguments");
	ULogger::flush();
	ULogger::setBuffered(false);
}

void Tests::testConversion()
{
	//printf("\n");
	//Logger::setType(Logger::kTypeConsole);

	std::string string;
	bool boolean;
	std::vector<char> charVector;
	char bytes[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
				        0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};
	std::string hexString = "000102030405060708090A0B0C0D0E0F00102030405060708090A0B0C0D0E0F0";
	std::string helloWorldHex = "48656c6c6f20776f726c6421"; // "Hello world!"
	std::string helloWorld = "Hello world!"; // "Hello world!"

	//void replaceChar(std::string & str, char before, char after)
	string = "";
	string = uReplaceChar(string, 'a', 'b');
	CPPUNIT_ASSERT( string.size() == 0 );
	string = "abcda";
	string = uReplaceChar(string, 'a', 'b');
	CPPUNIT_ASSERT( string.size() == 5 );
	CPPUNIT_ASSERT( string.compare("bbcdb") == 0 );

	//std::string toUpperCase(const std::string & str)
	CPPUNIT_ASSERT( uToUpperCase("").compare("") == 0 );
	CPPUNIT_ASSERT( uToUpperCase("Hello World!").compare("HELLO WORLD!") == 0 );

	//std::string toLowerCase(const std::string & str)
	CPPUNIT_ASSERT( uToLowerCase("").compare("") == 0 );
	CPPUNIT_ASSERT( uToLowerCase("Hello World!").compare("hello world!") == 0 );

	//std::string number2str(const int & number)
	string = uNumber2Str(0);
	CPPUNIT_ASSERT( string.compare("0") == 0 );
	string = uNumber2Str(10);
	CPPUNIT_ASSERT( string.compare("10") == 0 );

	//std::string number2str(const float & number)
	string = uNumber2Str(0.0f);
	CPPUNIT_ASSERT( string.compare("0") == 0 );
	string = uNumber2Str(10.2030f);
	CPPUNIT_ASSERT( string.compare("10.203") == 0 || string.compare("10,203") == 0 );

	//std::string number2str(const double & number)
	string = uNumber2Str(0.0);
	CPPUNIT_ASSERT( string.compare("0") == 0 );
	string = uNumber2Str(10.2030);
	CPPUNIT_ASSERT( string.compare("10.203") == 0 || string.compare("10,203") == 0 );

	//std::string bool2str(const bool & boolean)
	string = uBool2Str(false);
	CPPUNIT_ASSERT( string.compare("false") == 0 );
	string = uBool2Str(true);
	CPPUNIT_ASSERT( string.compare("true") == 0 );

	//bool str2Bool(const char * str)
	boolean = uStr2Bool(0);
	CPPUNIT_ASSERT( boolean == true );
	boolean = uStr2Bool("false");
	CPPUNIT_ASSERT( boolean == false );
	boolean = uStr2Bool("FALSE");
	CPPUNIT_ASSERT( boolean == false );
	boolean = uStr2Bool("0");
	CPPUNIT_ASSERT( boolean == false );
	boolean = uStr2Bool("FalSe");
	CPPUNIT_ASSERT( boolean == true );
	boolean = uStr2Bool("true");
	CPPUNIT_ASSERT( boolean == true );
	boolean = uStr2Bool("");
	CPPUNIT_ASSERT( boolean == true );

	//std::string bytes2Hex(const char * bytes, unsigned int bytesLen)
	string = uBytes2Hex(0, 0);
	CPPUNIT_ASSERT( string.empty() );
	string = uBytes2Hex(bytes, 16*2);
	CPPUNIT_ASSERT_MESSAGE(string, string.compare(hexString) == 0 );

	//std::vector<char> hex2bytes(const std::string & hex)
	charVector = uHex2Bytes(hexString);
	CPPUNIT_ASSERT( charVector.size() == 16*2 );
	for(unsigned int i=0; i<charVector.size(); ++i)
	{
		CPPUNIT_ASSERT(charVector[i] == bytes[i]);
	}

	//std::vector<char> hex2bytes(const char * hex, int hexLen)
	charVector = uHex2Bytes(0, 0);
	CPPUNIT_ASSERT( charVector.size() == 0 );
	charVector = uHex2Bytes(&hexString[0], hexString.size());
	CPPUNIT_ASSERT( charVector.size() == 16*2 );
	for(unsigned int i=0; i<charVector.size(); ++i)
	{
		CPPUNIT_ASSERT( charVector[i] == bytes[i] );
	}

	//std::string hex2str(const std::string & hex)
	charVector = uHex2Bytes(helloWorldHex.c_str()); // "Hello world!"
	CPPUNIT_ASSERT( charVector.size() ==  helloWorld.size());
	for(unsigned int i=0; i<charVector.size(); ++i)
	{
		CPPUNIT_ASSERT( charVector[i] == helloWorld[i]);
	}

	//unsigned char hex2ascii(const unsigned char & c, bool rightPart)
	unsigned char ascii;
	for(int c=0; c<256; ++c)
	{
		ascii = uHex2Ascii(static_cast<unsigned char>(c), true);
		CPPUNIT_ASSERT( ascii == HEX2ASCII[c][1] );
		ascii = uHex2Ascii(static_cast<unsigned char>(c), false);
		CPPUNIT_ASSERT( ascii == HEX2ASCII[c][0] );
	}

	//unsigned char ascii2hex(const unsigned char & c)
	unsigned char hex;
	hex = uAscii2Hex('0');
	CPPUNIT_ASSERT( hex == 0x00 );
	hex = uAscii2Hex('9');
	CPPUNIT_ASSERT( hex == 0x09 );
	hex = uAscii2Hex('a');
	CPPUNIT_ASSERT( hex == 0x0a );
	hex = uAscii2Hex('f');
	CPPUNIT_ASSERT( hex == 0x0f );
	hex = uAscii2Hex('A');
	CPPUNIT_ASSERT( hex == 0x0a );
	hex = uAscii2Hex('F');
	CPPUNIT_ASSERT( hex == 0x0f );
	hex = uAscii2Hex('Z');
	CPPUNIT_ASSERT( hex == 0x00 );
}

void Tests::testUtilStl()
{
	//printf("\n");
	//Logger::setType(Logger::kTypeConsole);

	std::list<int> list;
	std::list<int> list2;
	std::vector<int> vector;
	std::map<int,int> map;
	std::multimap<int,int> multimap;
	std::set<int> set;
	int val;

	//std::list<K> uniqueKeys(const std::multimap<K, V> & mm)
	multimap.clear();
	list = uUniqueKeys(multimap);
	CPPUNIT_ASSERT( list.size() == 0 );
	multimap.insert(std::pair<int,int>(1,10));
	multimap.insert(std::pair<int,int>(1,11));
	multimap.insert(std::pair<int,int>(2,20));
	list = uUniqueKeys(multimap);
	CPPUNIT_ASSERT( list.size() == 2 );
	CPPUNIT_ASSERT( *list.begin() == 1 );
	CPPUNIT_ASSERT( *(++list.begin()) == 2 );

	//std::vector<K> uKeys(const std::multimap<K, V> & mm)
	multimap.clear();
	vector = uKeys(multimap);
	CPPUNIT_ASSERT( vector.size() == 0 );
	multimap.insert(std::pair<int,int>(1,10));
	multimap.insert(std::pair<int,int>(1,11));
	multimap.insert(std::pair<int,int>(2,20));
	vector = uKeys(multimap);
	CPPUNIT_ASSERT( vector.size() == 3 );
	CPPUNIT_ASSERT( *vector.begin() == 1 );
	CPPUNIT_ASSERT( *(++vector.begin()) == 1 );
	CPPUNIT_ASSERT( *(++(++vector.begin())) == 2 );

	//std::vector<V> uValues(const std::multimap<K, V> & mm)
	multimap.clear();
	vector = uValues(multimap);
	CPPUNIT_ASSERT( vector.size() == 0 );
	multimap.insert(std::pair<int,int>(1,10));
	multimap.insert(std::pair<int,int>(1,11));
	multimap.insert(std::pair<int,int>(2,20));
	vector = uValues(multimap);
	CPPUNIT_ASSERT( vector.size() == 3 );
	CPPUNIT_ASSERT( *vector.begin() == 10 );
	CPPUNIT_ASSERT( *(++vector.begin()) == 11 );
	CPPUNIT_ASSERT( *(++(++vector.begin())) == 20 );

	//std::list<K> uKeysList(const std::multimap<K, V> & mm)
	multimap.clear();
	list = uKeysList(multimap);
	CPPUNIT_ASSERT( list.size() == 0 );
	multimap.insert(std::pair<int,int>(1,10));
	multimap.insert(std::pair<int,int>(1,11));
	multimap.insert(std::pair<int,int>(2,20));
	list = uKeysList(multimap);
	CPPUNIT_ASSERT( list.size() == 3 );
	CPPUNIT_ASSERT( *list.begin() == 1 );
	CPPUNIT_ASSERT( *(++list.begin()) == 1 );
	CPPUNIT_ASSERT( *(++(++list.begin())) == 2 );

	//std::list<V> uValuesList(const std::multimap<K, V> & mm)
	multimap.clear();
	list = uValuesList(multimap);
	CPPUNIT_ASSERT( list.size() == 0 );
	multimap.insert(std::pair<int,int>(1,10));
	multimap.insert(std::pair<int,int>(1,11));
	multimap.insert(std::pair<int,int>(2,20));
	list = uValuesList(multimap);
	CPPUNIT_ASSERT( list.size() == 3 );
	CPPUNIT_ASSERT( *list.begin() == 10 );
	CPPUNIT_ASSERT( *(++list.begin()) == 11 );
	CPPUNIT_ASSERT( *(++(++list.begin())) == 20 );

	//std::list<V> uValues(const std::multimap<K, V> & mm, const K & key)
	multimap.clear();
	list = uValues(multimap,1);
	CPPUNIT_ASSERT( list.size() == 0 );
	multimap.insert(std::pair<int,int>(1,10));
	multimap.insert(std::pair<int,int>(1,11));
	multimap.insert(std::pair<int,int>(2,20));
	list = uValues(multimap,1);
	CPPUNIT_ASSERT( list.size() == 2 );
	CPPUNIT_ASSERT( *list.begin() == 10 );
	CPPUNIT_ASSERT( *(++list.begin()) == 11 );

	//std::vector<K> uKeys(const std::map<K, V> & m)
	map.clear();
	vector = uKeys(map);
	CPPUNIT_ASSERT( vector.size() == 0 );
	map.insert(std::pair<int,int>(1,10));
	map.insert(std::pair<int,int>(2,20));
	vector = uKeys(map);
	CPPUNIT_ASSERT( vector.size() == 2 );
	CPPUNIT_ASSERT( vector[0] == 1 );
	CPPUNIT_ASSERT( vector[1] == 2 );

	//std::vector<V> uValues(const std::map<K, V> & m)
	map.clear();
	vector = uValues(map);
	CPPUNIT_ASSERT( vector.size() == 0 );
	map.insert(std::pair<int,int>(1,10));
	map.insert(std::pair<int,int>(2,20));
	vector = uValues(map);
	CPPUNIT_ASSERT( vector.size() == 2 );
	CPPUNIT_ASSERT( vector[0] == 10 );
	CPPUNIT_ASSERT( vector[1] == 20 );

	//V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
	map.clear();
	val = uValue(map, 1);
	CPPUNIT_ASSERT( val == 0 );
	val = uValue(map, 1, 2);
	CPPUNIT_ASSERT( val == 2 );
	map.insert(std::pair<int,int>(1,10));
	map.insert(std::pair<int,int>(2,20));
	val = uValue(map, 1);
	CPPUNIT_ASSERT( val == 10 );

	//V uTake(std::map<K, V> & m, const K & key, const V & defaultValue = V())
	map.clear();
	val = uTake(map, 1);
	CPPUNIT_ASSERT( val == 0 );
	val = uTake(map, 1, 2);
	CPPUNIT_ASSERT( val == 2 );
	map.insert(std::pair<int,int>(1,10));
	map.insert(std::pair<int,int>(2,20));
	val = uTake(map, 1);
	CPPUNIT_ASSERT( map.size() == 1 );
	CPPUNIT_ASSERT( val == 10 );
	CPPUNIT_ASSERT( (*map.begin()).first == 2 );

	//std::list<V>::iterator iteratorAt(std::list<V> & list, const unsigned int & pos)
	list.clear();
	CPPUNIT_ASSERT( uIteratorAt(list, 0) == list.end() );
	list.push_back(0);
	list.push_back(1);
	list.push_back(2);
	CPPUNIT_ASSERT( uIteratorAt(list, 0) == list.begin() );
	CPPUNIT_ASSERT( uIteratorAt(list, 3) == list.end() );

	//std::vector<V>::iterator iteratorAt(std::vector<V> & v, const unsigned int & pos)

	//V & uValueAt(std::list<V> & list, const unsigned int & pos)
	CPPUNIT_ASSERT( uValueAt(list, 0) == 0 );
	CPPUNIT_ASSERT( uValueAt(list, 1) == 1 );
	CPPUNIT_ASSERT( uValueAt(list, 2) == 2 );

	//bool contains(const std::list<V> & list, const V & uValue)
	list.clear();
	CPPUNIT_ASSERT( uContains(list, 1) == false );
	list.push_back(1);
	CPPUNIT_ASSERT( uContains(list, 1) == true );

	//bool contains(const std::map<K, V> & map, const K & key)
	map.clear();
	CPPUNIT_ASSERT( uContains(map, 1) == false );
	map.insert(std::pair<int,int>(1,10));
	CPPUNIT_ASSERT( uContains(map, 1) == true );

	//bool contains(const std::multimap<K, V> & map, const K & key)
	multimap.clear();
	CPPUNIT_ASSERT( uContains(multimap, 1) == false );
	multimap.insert(std::pair<int,int>(1,10));
	CPPUNIT_ASSERT( uContains(multimap, 1) == true );

	//bool contains(const std::multimap<K, V> & map, const K & key)
	set.clear();
	CPPUNIT_ASSERT( uContains(set, 1) == false );
	set.insert(1);
	CPPUNIT_ASSERT( uContains(set, 1) == true );

	//void uInsert(std::map<K, V> & map, const std::pair<K, V> & pair)
	map.clear();
	map.insert(std::pair<int, int>(1, 2));
	map.insert(std::pair<int, int>(1, 3));
	CPPUNIT_ASSERT( map.size() == 1 && uValue(map, 1) == 2 );
	uInsert(map, std::pair<int, int>(1, 3));
	CPPUNIT_ASSERT( map.size() == 1 && uValue(map, 1) == 3 );

	//std::vector<V> uListToVector(const std::list<V> & list)
	list.clear();
	vector = uListToVector(list);
	CPPUNIT_ASSERT( vector.size() == 0 );
	list.push_back(1);
	vector = uListToVector(list);
	CPPUNIT_ASSERT( vector.size() == 1 );
	CPPUNIT_ASSERT( vector[0] == 1 );

	//std::list<V> uVectorToList(const std::vector<V> & v)
	vector.clear();
	list = uVectorToList(vector);
	CPPUNIT_ASSERT( list.size() == 0 );
	vector.push_back(1);
	list = uVectorToList(vector);
	CPPUNIT_ASSERT( list.size() == 1 );
	CPPUNIT_ASSERT( *list.begin() == 1 );

	//void append(std::list<V> & list, const std::list<V> & newItems)
	list.clear();
	list2.clear();
	uAppend(list, list2);
	CPPUNIT_ASSERT( list.size() == 0 );
	list2.push_back(1);
	uAppend(list, list2);
	CPPUNIT_ASSERT( list.size() == 1 );

	//int uIndexOf(const std::vector<V> & list, const V & uValue)
	vector.clear();
	val = uIndexOf(vector, 1);
	CPPUNIT_ASSERT( val < 0 );
	vector.push_back(1);
	vector.push_back(2);
	val = uIndexOf(vector, 1);
	CPPUNIT_ASSERT( val == 0 );
	val = uIndexOf(vector, 2);
	CPPUNIT_ASSERT( val == 1 );
	val = uIndexOf(vector, 3);
	CPPUNIT_ASSERT( val < 0 );

	// std::vector<std::string> uSplit(const std::string & str, char separator = ' ')
	std::list<std::string> v = uSplit("Hello the world!", ' ');
	CPPUNIT_ASSERT( v.size() == 3 );
	CPPUNIT_ASSERT( uValueAt(v,0).compare("Hello") == 0 );
	CPPUNIT_ASSERT( uValueAt(v,1).compare("the") == 0 );
	CPPUNIT_ASSERT( uValueAt(v,2).compare("world!") == 0 );

	// bool uIsDigit(const char c)
	CPPUNIT_ASSERT( uIsDigit('0') );
	CPPUNIT_ASSERT( uIsDigit('1') );
	CPPUNIT_ASSERT( uIsDigit('2') );
	CPPUNIT_ASSERT( uIsDigit('3') );
	CPPUNIT_ASSERT( uIsDigit('4') );
	CPPUNIT_ASSERT( uIsDigit('5') );
	CPPUNIT_ASSERT( uIsDigit('6') );
	CPPUNIT_ASSERT( uIsDigit('7') );
	CPPUNIT_ASSERT( uIsDigit('8') );
	CPPUNIT_ASSERT( uIsDigit('9') );
	CPPUNIT_ASSERT( !uIsDigit('0'-1) );
	CPPUNIT_ASSERT( !uIsDigit('9'+1) );

	// std::list<std::string> uSplitNumChar(const std::string & str)
	v = uSplitNumChar("Hello 03 my 65 world!");
	CPPUNIT_ASSERT( v.size() == 5 );
	CPPUNIT_ASSERT( uValueAt(v,0).compare("Hello ") == 0 );
	CPPUNIT_ASSERT( uValueAt(v,1).compare("03") == 0 );
	CPPUNIT_ASSERT( uValueAt(v,2).compare(" my ") == 0 );
	CPPUNIT_ASSERT( uValueAt(v,3).compare("65") == 0 );
	CPPUNIT_ASSERT( uValueAt(v,4).compare(" world!") == 0 );

	// int uStrNumCmp(const std::string & a, const std::string & b)
	CPPUNIT_ASSERT(uStrNumCmp("Image9.jpg", "Image10.jpg") == -1);

	//bool uStrContains(const std::string & string, const std::string & substring)
	CPPUNIT_ASSERT(uStrContains("hello_world", "hello"));
	CPPUNIT_ASSERT(uStrContains("hello_world", "world"));
	CPPUNIT_ASSERT(!uStrContains("hello_world", "alpha"));
	CPPUNIT_ASSERT(!uStrContains("hello_world", "hello_world2"));
}

void Tests::testDirectory()
{
	//bool UDir::dirExists(const char * dirPath);
	CPPUNIT_ASSERT( UDir::exists("bad/directory/") == false );
	CPPUNIT_ASSERT( UDir::exists("./") == true );
#if WIN32
	CPPUNIT_ASSERT( UDir::exists(".\\") == true );
#endif
	CPPUNIT_ASSERT( UDir::exists(UDir::currentDir().c_str()) == true );

	//std::string UDir::getDir(const char * filePath);
	CPPUNIT_ASSERT( UDir::getDir("a/directory/file").compare("a/directory") == 0 );
#if WIN32
	CPPUNIT_ASSERT(UDir::getDir("a\\directory\\file").compare("a\\directory") == 0 );
#endif
	CPPUNIT_ASSERT( UDir::getDir("file").compare(".") == 0 );
	CPPUNIT_ASSERT( UDir::getDir("").compare(".") == 0 );

	//std::string UDir::currentDir(bool trailingSeparator = false);

	//bool UDir::makeDir(const char * dirPath);
	CPPUNIT_ASSERT( UDir::makeDir("./LogTestUtil/dummyDirectory") == true );
	CPPUNIT_ASSERT( UDir::makeDir("./LogTestUtil/dummyDirectory") == false );

	//bool UDir::removeDir(const char * dirPath);
	CPPUNIT_ASSERT( UDir::removeDir("./LogTestUtil/dummyDirectory") == true );
	CPPUNIT_ASSERT( UDir::removeDir("./LogTestUtil/dummyDirectory") == false );

	UDir dir("./data", "x");
	std::vector<std::string> fileNames = uListToVector(uSplit("1.x 2.x 10.x 11.x 101.x", ' '));
	unsigned int i=0;
	CPPUNIT_ASSERT(dir.getFileNames().size() == 5);
	for(; i<fileNames.size(); ++i)
	{
		std::string next = dir.getNextFileName();
		CPPUNIT_ASSERT(fileNames[i].compare(next) == 0);
	}
	dir.update();
	CPPUNIT_ASSERT(dir.getNextFileName().compare("") == 0);
	CPPUNIT_ASSERT(i == 5);
}

void Tests::testFile()
{
	ULogger::setType(ULogger::kTypeFile, "LogTestUtil/testFile.txt", false);
	ULogger::reset(); // this will close the file

	//bool UTIL_EXP UFile::exists(const std::string &filePath);
	CPPUNIT_ASSERT( UFile::exists("./LogTestUtil/dummyFile") == false );
	CPPUNIT_ASSERT( UFile::exists("LogTestUtil/testFile.txt") == true );
#ifdef WIN32
	CPPUNIT_ASSERT( UFile::exists("LogTestUtil\\testFile.txt") == true );
#endif

	UFile::copy("LogTestUtil/testFile.txt", "LogTestUtil/testFileCpy.txt");
	CPPUNIT_ASSERT( UFile::exists("LogTestUtil/testFileCpy.txt") == true );
	UFile::erase("LogTestUtil/testFileCpy.txt");
	CPPUNIT_ASSERT( UFile::exists("LogTestUtil/testFileCpy.txt") == false );

	//long UTIL_EXP fileLength(const std::string &filePath);

	//int UTIL_EXP eraseFile(const std::string &filePath);

	//int UTIL_EXP renameFile(const std::string &oldFilePath, const std::string &newFilePath);

	//std::string UFile::getName(const std::string & filePath, bool withExtension);
	CPPUNIT_ASSERT( UFile::getName("").compare("") == 0 );
	CPPUNIT_ASSERT( UFile::getName("testFile").compare("testFile") == 0 );
	CPPUNIT_ASSERT( UFile::getName("./LogTestUtil/testFile.txt").compare("testFile.txt") == 0 );
	CPPUNIT_ASSERT( UFile::getName("./LogTestUtil/testFile.txt", false).compare("testFile") == 0 );
	CPPUNIT_ASSERT( UFile::getName("./LogTestUtil/testFile.txt.png", false).compare("testFile.txt") == 0 );
#ifdef WIN32
	CPPUNIT_ASSERT( UFile::getName(".\\LogTestUtil\\testFile.txt").compare("testFile.txt") == 0 );
#endif

	//std::string UFile::getExtension(const std::string & filePath);
	CPPUNIT_ASSERT( UFile::getExtension("").compare("") == 0 );
	CPPUNIT_ASSERT( UFile::getExtension("testFile").compare("") == 0 );
	CPPUNIT_ASSERT( UFile::getExtension("testFile.txt").compare("txt") == 0 );
	CPPUNIT_ASSERT( UFile::getExtension("./testFile.txt.png").compare("png") == 0 );
}

// TODO : some tests needed...
void Tests::testMathFunctions()
{
	ULogger::setType(ULogger::kTypeFile, "LogTestUtil/testMathFunctions.txt", false);

	//inline std::vector<T> uXMatch(const T * vA, const T * vB, unsigned int sizeA, unsigned int sizeB, UXMatchMethod method)
	//inline T uXMatch(const T * vA, const T * vB, unsigned int sizeA, unsigned int sizeB, unsigned int index, UXMatchMethod method)
	float x1[] = {1, 2, 5};
	float x2[] = {2, 5, 1};
	std::vector<float> r;
	r = uXMatch(x1, x2, 3, 3, UXCorrRaw);
	CPPUNIT_ASSERT(r.size() == 3+3-1);
	CPPUNIT_ASSERT(r[0] == 1.0f);
	CPPUNIT_ASSERT(r[1] == 7.0f);
	CPPUNIT_ASSERT(r[2] == 17.0f);
	CPPUNIT_ASSERT(r[3] == 29.0f);
	CPPUNIT_ASSERT(r[4] == 10.0f);
	r = uXMatch(x1, x2, 3, 3, UXCorrBiased);
	CPPUNIT_ASSERT(r[0] >= 0.32f && r[0] <= 0.34f);
	CPPUNIT_ASSERT(r[1] >= 2.32f && r[1] <= 2.34f);
	CPPUNIT_ASSERT(r[2] >= 5.65f && r[2] <= 5.67f);
	CPPUNIT_ASSERT(r[3] >= 9.65f && r[3] <= 9.67f);
	CPPUNIT_ASSERT(r[4] >= 3.32f && r[4] <= 3.34f);
	r = uXMatch(x1, x2, 3, 3, UXCorrUnbiased);
	CPPUNIT_ASSERT(r[0] == 1.0f);
	CPPUNIT_ASSERT(r[1] == 3.5f);
	CPPUNIT_ASSERT(r[2] >= 5.65f && r[2] <= 5.67f);
	CPPUNIT_ASSERT(r[3] == 14.5f);
	CPPUNIT_ASSERT(r[4] == 10.0f);
	r = uXMatch(x1, x2, 3, 3, UXCorrCoeff);
	CPPUNIT_ASSERT(r[0] >= 0.02f && r[0] <= 0.04f);
	CPPUNIT_ASSERT(r[1] >= 0.22f && r[1] <= 0.24f);
	CPPUNIT_ASSERT(r[2] >= 0.55f && r[2] <= 0.57f);
	CPPUNIT_ASSERT(r[3] >= 0.95f && r[3] <= 0.97f);
	CPPUNIT_ASSERT(r[4] >= 0.32f && r[4] <= 0.34f);
	r = uXMatch(x1, x2, 3, 3, UXCovRaw);
	CPPUNIT_ASSERT(r[0] >= 2.76f && r[0] <= 2.78f);
	CPPUNIT_ASSERT(r[1] >= -2.78f && r[1] <= -2.76f);
	CPPUNIT_ASSERT(r[2] >= -4.34f && r[2] <= -4.32f);
	CPPUNIT_ASSERT(r[3] >= 5.87f && r[3] <= 5.89f);
	CPPUNIT_ASSERT(r[4] >= -1.56f && r[4] <= -1.54f);
	r = uXMatch(x1, x2, 3, 3, UXCovBiased);
	CPPUNIT_ASSERT(r[0] >= 0.91f && r[0] <= 0.93f);
	CPPUNIT_ASSERT(r[1] >= -0.93f && r[1] <= -0.91f);
	CPPUNIT_ASSERT(r[2] >= -1.45f && r[2] <= -1.43f);
	CPPUNIT_ASSERT(r[3] >= 1.95f && r[3] <= 1.97f);
	CPPUNIT_ASSERT(r[4] >= -0.52f && r[4] <= -0.50f);
	r = uXMatch(x1, x2, 3, 3, UXCovUnbiased);
	CPPUNIT_ASSERT(r[0] >= 2.76f && r[0] <= 2.78f);
	CPPUNIT_ASSERT(r[1] >= -1.39f && r[1] <= -1.37f);
	CPPUNIT_ASSERT(r[2] >= -1.45f && r[2] <= -1.43f);
	CPPUNIT_ASSERT(r[3] >= 2.93f && r[3] <= 2.95f);
	CPPUNIT_ASSERT(r[4] >= -1.56f && r[4] <= -1.54f);
	r = uXMatch(x1, x2, 3, 3, UXCovCoeff);
	CPPUNIT_ASSERT(r[0] >= 0.31f && r[0] <= 0.33f);
	CPPUNIT_ASSERT(r[1] >= -0.33f && r[1] <= -0.31f);
	CPPUNIT_ASSERT(r[2] >= -0.51f && r[2] <= -0.49);
	CPPUNIT_ASSERT(r[3] >= 0.66 && r[3] <= 0.68f);
	CPPUNIT_ASSERT(r[4] >= -0.18 && r[4] <= -0.16f);
	//
	float x3[] = {1, 2, 5, 0, 1};
	float x4[] = {2, 5, 1};
	float x33[] = {1, 0, 5, 2, 1};
	float x44[] = {1, 5, 2};

	std::vector<float> r1, r2, r3, r4;
	UXMatchMethod method = UXCovUnbiased;

	r1 = uXMatch(x3, x4, 5, 3, method);
	CPPUNIT_ASSERT(r1.size() == 5+3-1);
	r2.resize(r1.size());
	CPPUNIT_ASSERT((r2[0] = uXMatch(x3, x4, 5, 3, 0, method)) >= r1[0]-0.001f && r2[0] <= r1[0]+0.001f);
	CPPUNIT_ASSERT((r2[1] = uXMatch(x3, x4, 5, 3, 1, method)) >= r1[1]-0.001f && r2[1] <= r1[1]+0.001f);
	CPPUNIT_ASSERT((r2[2] = uXMatch(x3, x4, 5, 3, 2, method)) >= r1[2]-0.001f && r2[2] <= r1[2]+0.001f);
	CPPUNIT_ASSERT((r2[3] = uXMatch(x3, x4, 5, 3, 3, method)) >= r1[3]-0.001f && r2[3] <= r1[3]+0.001f);
	CPPUNIT_ASSERT((r2[4] = uXMatch(x3, x4, 5, 3, 4, method)) >= r1[4]-0.001f && r2[4] <= r1[4]+0.001f);
	CPPUNIT_ASSERT((r2[5] = uXMatch(x3, x4, 5, 3, 5, method)) >= r1[5]-0.001f && r2[5] <= r1[5]+0.001f);
	CPPUNIT_ASSERT((r2[6] = uXMatch(x3, x4, 5, 3, 6, method)) >= r1[6]-0.001f && r2[6] <= r1[6]+0.001f);

	r3 = uXMatch(x33, x44, 5, 3, method);
	CPPUNIT_ASSERT(r3.size() == 5+3-1);
	r4.resize(r3.size());
	CPPUNIT_ASSERT((r4[0] = uXMatch(x33, x44, 5, 3, 0, method)) >= r3[0]-0.001f && r2[0] <= r1[0]+0.001f);
	CPPUNIT_ASSERT((r4[1] = uXMatch(x33, x44, 5, 3, 1, method)) >= r3[1]-0.001f && r2[1] <= r1[1]+0.001f);
	CPPUNIT_ASSERT((r4[2] = uXMatch(x33, x44, 5, 3, 2, method)) >= r3[2]-0.001f && r2[2] <= r1[2]+0.001f);
	CPPUNIT_ASSERT((r4[3] = uXMatch(x33, x44, 5, 3, 3, method)) >= r3[3]-0.001f && r2[3] <= r1[3]+0.001f);
	CPPUNIT_ASSERT((r4[4] = uXMatch(x33, x44, 5, 3, 4, method)) >= r3[4]-0.001f && r2[4] <= r1[4]+0.001f);
	CPPUNIT_ASSERT((r4[5] = uXMatch(x33, x44, 5, 3, 5, method)) >= r3[5]-0.001f && r2[5] <= r1[5]+0.001f);
	CPPUNIT_ASSERT((r4[6] = uXMatch(x33, x44, 5, 3, 6, method)) >= r3[6]-0.001f && r2[6] <= r1[6]+0.001f);
	CPPUNIT_ASSERT(r3[0] == r1[6]);
	CPPUNIT_ASSERT(r3[1] == r1[5]);
	CPPUNIT_ASSERT(r3[2] == r1[4]);
	CPPUNIT_ASSERT(r3[3] == r1[3]);
	CPPUNIT_ASSERT(r3[4] == r1[2]);
	CPPUNIT_ASSERT(r3[5] == r1[1]);
	CPPUNIT_ASSERT(r3[6] == r1[0]);

	r1 = uXMatch(x4, x3, 3, 5, method);
	CPPUNIT_ASSERT(r1.size() == 5+3-1);
	r2.resize(r1.size());
	CPPUNIT_ASSERT((r2[0] = uXMatch(x4, x3, 3, 5, 0, method)) >= r1[0]-0.001f && r2[0] <= r1[0]+0.001f);
	CPPUNIT_ASSERT((r2[1] = uXMatch(x4, x3, 3, 5, 1, method)) >= r1[1]-0.001f && r2[1] <= r1[1]+0.001f);
	CPPUNIT_ASSERT((r2[2] = uXMatch(x4, x3, 3, 5, 2, method)) >= r1[2]-0.001f && r2[2] <= r1[2]+0.001f);
	CPPUNIT_ASSERT((r2[3] = uXMatch(x4, x3, 3, 5, 3, method)) >= r1[3]-0.001f && r2[3] <= r1[3]+0.001f);
	CPPUNIT_ASSERT((r2[4] = uXMatch(x4, x3, 3, 5, 4, method)) >= r1[4]-0.001f && r2[4] <= r1[4]+0.001f);
	CPPUNIT_ASSERT((r2[5] = uXMatch(x4, x3, 3, 5, 5, method)) >= r1[5]-0.001f && r2[5] <= r1[5]+0.001f);
	CPPUNIT_ASSERT((r2[6] = uXMatch(x4, x3, 3, 5, 6, method)) >= r1[6]-0.001f && r2[6] <= r1[6]+0.001f);

	r3 = uXMatch(x44, x33, 3, 5, method);
	CPPUNIT_ASSERT(r3.size() == 5+3-1);
	r4.resize(r3.size());
	CPPUNIT_ASSERT((r4[0] = uXMatch(x44, x33, 3, 5, 0, method)) >= r3[0]-0.001f && r4[0] <= r3[0]+0.001f);
	CPPUNIT_ASSERT((r4[1] = uXMatch(x44, x33, 3, 5, 1, method)) >= r3[1]-0.001f && r4[1] <= r3[1]+0.001f);
	CPPUNIT_ASSERT((r4[2] = uXMatch(x44, x33, 3, 5, 2, method)) >= r3[2]-0.001f && r4[2] <= r3[2]+0.001f);
	CPPUNIT_ASSERT((r4[3] = uXMatch(x44, x33, 3, 5, 3, method)) >= r3[3]-0.001f && r4[3] <= r3[3]+0.001f);
	CPPUNIT_ASSERT((r4[4] = uXMatch(x44, x33, 3, 5, 4, method)) >= r3[4]-0.001f && r4[4] <= r3[4]+0.001f);
	CPPUNIT_ASSERT((r4[5] = uXMatch(x44, x33, 3, 5, 5, method)) >= r3[5]-0.001f && r4[5] <= r3[5]+0.001f);
	CPPUNIT_ASSERT((r4[6] = uXMatch(x44, x33, 3, 5, 6, method)) >= r3[6]-0.001f && r4[6] <= r3[6]+0.001f);
	CPPUNIT_ASSERT(r3[0] == r1[6]);
	CPPUNIT_ASSERT(r3[1] == r1[5]);
	CPPUNIT_ASSERT(r3[2] == r1[4]);
	CPPUNIT_ASSERT(r3[3] == r1[3]);
	CPPUNIT_ASSERT(r3[4] == r1[2]);
	CPPUNIT_ASSERT(r3[5] == r1[1]);
	CPPUNIT_ASSERT(r3[6] == r1[0]);

	//

	std::vector<float> vF(4);
	unsigned int index = 0;
	float mi = 0;
	float ma = 0;
	unsigned int miIndex = 0;
	unsigned int maIndex = 0;
	// {1,2,3,4}
	vF[0] = -1;
	vF[1] = -1;
	vF[2] = 7;
	vF[3] = 7;
	std::list<float> lF = uVectorToList(vF);
	float m = 3;
	int s = 4618;
	float theSum = 12;
	float max = 7;
	float min = -1;
	float maxIndex = 2;
	float minIndex = 0;

	//inline T uMax(const T * v, unsigned int size, unsigned int & index = 0)
	CPPUNIT_ASSERT( uMax(vF.data(), vF.size()) == max );
	CPPUNIT_ASSERT( uMax(vF.data(), vF.size(), index) == max );
	CPPUNIT_ASSERT( index == maxIndex );

	CPPUNIT_ASSERT( uMin(vF.data(), vF.size()) == min );
	CPPUNIT_ASSERT( uMin(vF.data(), vF.size(), index) == min );
	CPPUNIT_ASSERT( index == minIndex );

	uMinMax(vF.data(), vF.size(), mi, ma);
	CPPUNIT_ASSERT( mi == min && ma == max );
	uMinMax(vF.data(), vF.size(), mi, ma, miIndex, maIndex);
	CPPUNIT_ASSERT( mi == min && ma == max && miIndex == minIndex && maIndex == maxIndex);

	//inline int uSign(const T & v)
	CPPUNIT_ASSERT( uSign(-1) == -1 );
	CPPUNIT_ASSERT( uSign(0) == 1 );
	CPPUNIT_ASSERT( uSign(1) == 1 );

	//inline T uSum(const std::list<T> & list)
	CPPUNIT_ASSERT(uSum(std::list<float>()) == 0);
	CPPUNIT_ASSERT(uSum(lF) == theSum);

	//inline T uSum(const std::vector<T> & v)
	CPPUNIT_ASSERT(uSum(std::vector<float>()) == 0);
	CPPUNIT_ASSERT(uSum(vF) == theSum);

	//inline T uSum(const T * v, unsigned int size)
	CPPUNIT_ASSERT(uSum((float *)0, 0) == 0);
	CPPUNIT_ASSERT(uSum(vF.data(), 0) == 0);
	CPPUNIT_ASSERT(uSum(vF) == theSum);

	//inline T uMean(const T * v, unsigned int size)
	CPPUNIT_ASSERT(uMean((float *)0,0) == 0);
	CPPUNIT_ASSERT(uMean(vF.data(),0) == 0);
	CPPUNIT_ASSERT(uMean(vF.data(), vF.size()) == m);

	//inline T uMean(const std::list<T> & list)
	CPPUNIT_ASSERT(uMean(std::list<float>()) == 0);
	CPPUNIT_ASSERT(uMean(lF) == m);

	//inline T uMean(const std::vector<T> & v)
	CPPUNIT_ASSERT(uMean(std::vector<float>()) == 0);
	CPPUNIT_ASSERT(uMean(vF) == m);

	//inline T uMeanSquaredError(const T * x, unsigned int sizeX, const T * y, unsigned int sizeY)
	CPPUNIT_ASSERT(uMeanSquaredError(x1, 3, x2, 3) >= 26.0f/3.0f-0.001f && uMeanSquaredError(x1, 3, x2, 3) <= 26.0f/3.0f+0.001f);
	CPPUNIT_ASSERT(uMeanSquaredError(x1, 3, x2, 2) == -1.0f);

	//inline T stdDev(const T * v, unsigned int size)
	CPPUNIT_ASSERT(uStdDev((float *)0, 0) == 0);
	CPPUNIT_ASSERT(uStdDev(vF.data(), 0) == 0);
	CPPUNIT_ASSERT(int(uStdDev(vF.data(), vF.size())*1000) == s);

	//inline T stdDev(const std::list<T> & list, const T & m)
	CPPUNIT_ASSERT(uStdDev(std::list<float>(), uMean(lF)) == 0);
	CPPUNIT_ASSERT(int(uStdDev(lF, uMean(lF))*1000) == s);

	//inline T stdDev(const T * v, unsigned int size, T meanV)
	CPPUNIT_ASSERT(uStdDev((float *)0, 0, uMean(vF.data(), vF.size())) == 0);
	CPPUNIT_ASSERT(uStdDev(vF.data(), 0, uMean(vF.data(), vF.size())) == 0);
	CPPUNIT_ASSERT(int(uStdDev(vF.data(), vF.size(), uMean(vF.data(), vF.size()))*1000) == s);

	//inline T stdDev(const std::vector<T> & v, const T & m)
	CPPUNIT_ASSERT(uStdDev(std::vector<float>(), uMean(vF)) == 0);
	CPPUNIT_ASSERT(int(uStdDev(vF, uMean(vF))*1000) == s);

	vF[0]=3.0f;
	vF[1]=4.0f;
	vF[2]=0.0f;
	vF[3]=0.0f;
	//inline float uNorm(const std::vector<float> & v)
	CPPUNIT_ASSERT(uNorm(vF) == 5.0f);
	//inline std::vector<float> uNormalize(const std::vector<float> & v)
	vF = uNormalize(vF);
	float rf = uNorm(vF);
	CPPUNIT_ASSERT(rf >= 0.99f && rf <= 1.01f);

	//inline std::list<unsigned int> uLocalMaxima(const T * v, unsigned int size)
	int maximaV1[] = {1, 0, 0, 0, 2, 3, 4, 3, 4, 5, 0, 1};
	int maximaV2[] = {0, 1, 0, 0, 0, 1, 0};
	int maximaV3[] = {1, 1, 1};
	int maximaV4[] = {0, 1};
	int maximaV5[] = {1};
	std::vector<int> maximaV6;
	CPPUNIT_ASSERT(uLocalMaxima(maximaV1, 12).size() == 4);
	CPPUNIT_ASSERT(uLocalMaxima(maximaV2, 7).size() == 2);
	CPPUNIT_ASSERT(uLocalMaxima(maximaV3, 3).size() == 0);
	CPPUNIT_ASSERT(uLocalMaxima(maximaV4, 2).size() == 1);
	CPPUNIT_ASSERT(uLocalMaxima(maximaV5, 1).size() == 1);
	CPPUNIT_ASSERT(uLocalMaxima(maximaV6.data(), 0).size() == 0);

	//std::vector<T> uIncrementedArray(const T & start, const T & increment, unsigned int count)
	//std::vector<T> uInvertArray(const std::vector<T> & v)
	std::vector<int> increment = uIncrementedArray(1, 2, 3);
	int incrementResult[3] = {1, 3, 5};
	CPPUNIT_ASSERT(increment[0] == incrementResult[0]);
	CPPUNIT_ASSERT(increment[1] == incrementResult[1]);
	CPPUNIT_ASSERT(increment[2] == incrementResult[2]);

	std::vector<int> invert = uInvertArray(increment);
	CPPUNIT_ASSERT(invert[0] == increment[2]);
	CPPUNIT_ASSERT(invert[1] == increment[1]);
	CPPUNIT_ASSERT(invert[2] == increment[0]);

	//bool uIsInBounds(const T& value, const T& low, const T& high)
	CPPUNIT_ASSERT(uIsInBounds(1, 0, 2));
	CPPUNIT_ASSERT(uIsInBounds(0, 0, 2));
	CPPUNIT_ASSERT(uIsInBounds(2, 0, 2));
	CPPUNIT_ASSERT(!uIsInBounds(-1, 0, 2));
	CPPUNIT_ASSERT(!uIsInBounds(3, 0, 2));
}

void Tests::testTimer()
{
	UTimer timer;
	double uValue;

	CPPUNIT_ASSERT(timer.now() > 0.0);

	double timeA = timer.now();
	uSleep(1000);
	double timeB = timer.now();
	uSleep(1000);
	double timeC = timer.now();

	//printf("timeA=%fs, timeB=%fs, timeC=%fs", timeA, timeB, timeC);
	CPPUNIT_ASSERT(timeB > timeA+0.9);
	CPPUNIT_ASSERT(timeC > timeB+0.9);

	//start/stop/getInterval
	timer.start();
	uSleep(100);
	timer.stop();
	uValue = timer.elapsed();
	CPPUNIT_ASSERT(uValue >= 0.09 && uValue < 0.11);

	//ticks
	timer.start();
	uSleep(100);
	uValue = timer.ticks();
	CPPUNIT_ASSERT(uValue >= 0.09 && uValue < 0.11);
}

void Tests::testObjDeletionThread()
{
	UObjDeletionThread<int> delThread(new int(1));
	delThread.startDeletion();
	delThread.setObj(new int(2));
}
