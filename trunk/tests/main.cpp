#include <cppunit/BriefTestProgressListener.h>
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/TestResult.h>
#include <cppunit/TestResultCollector.h>
#include <cppunit/TestRunner.h>

#include <iostream>
#include <fstream>
#include "ULogger.h"
#ifdef _MSC_VER
// Use Visual C++'s memory checking functionality
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif // _MSC_VER

int main( int argc, char **argv)
{    
    #ifdef _MSC_VER
    //_crtBreakAlloc = 189;
    _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF ); 
    #endif // _MSC_VER

    //Header of the test
    char header[150] = "-------------------------------------\n"
                       "Util Test\n"
                       "-------------------------------------\n";
    CPPUNIT_NS::stdCOut() << header;

    // Create the event manager and test controller
    CPPUNIT_NS::TestResult controller;

    // Add a listener that colllects test result
    CPPUNIT_NS::TestResultCollector result;
    controller.addListener(&result);

    // Add a listener that print dots as test run.
    CPPUNIT_NS::BriefTestProgressListener progress;
    controller.addListener(&progress);      

    // Add the top suite to the test runner
    CPPUNIT_NS::TestRunner runner;
    runner.addTest(CPPUNIT_NS::TestFactoryRegistry::getRegistry().makeTest());
    runner.run(controller);

    // Print test in a compiler compatible format.
    CPPUNIT_NS::CompilerOutputter outputter( &result, CPPUNIT_NS::stdCOut() );
    outputter.write(); 

    // If a path is passed in argument copy the result in it
    if (argc == 2)
    {
        std::ofstream myfile;
        myfile.open (argv[1]);
        CPPUNIT_NS::CompilerOutputter outputterFile( &result, myfile);
        myfile << header;
        outputterFile.write(); 
        myfile.close();
    }

    return 0;
}
