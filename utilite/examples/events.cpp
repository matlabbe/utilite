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

#include <utilite/UEventsManager.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEvent.h>
#include <utilite/ULogger.h>

// UtiLite.h can be used if we don't want to include
// each separated objects.
//#include <UtiLite.h>

// Create a custom event, called SpecialEvent.
class SpecialEvent : public UEvent
{
public:
   SpecialEvent(int code) : UEvent(code) {}
   ~SpecialEvent() {}

   // Inherited method from UEvent.
   virtual std::string getClassName() const {
	   return "SpecialEvent";
   }
};

// Create an events handler.
class EventsPrinter : public UEventsHandler
{
public:
   EventsPrinter() {}
   ~EventsPrinter() {}

protected:
   // Inherited method from UEventsHandler.
   virtual void handleEvent(UEvent * e) {
      // Only handle SpecialEvent type
      if(e->getClassName().compare("SpecialEvent") == 0) {
         // Logger syntax is the same as a "printf()".
         UINFO("SpecialEvent \"%d\" received!", e->getCode());
      }
   }
};

int main(int argc, char * argv[])
{
   // Set the logger type. The choices are kTypeConsole,
   // kTypeFile or kTypeNoLog (nothing is logged).
   ULogger::setType(ULogger::kTypeConsole);

   // Set the logger severity level (kDebug, kInfo, kWarning, kError).
   // All log entries under the severity level are not logged. Here,
   // only debug messages are not logged.
   ULogger::setLevel(ULogger::kInfo);

   // Use a predefined Macro to easy logging. It can be
   // called anywhere in the application as the logger is
   // a Singleton.
   UDEBUG("This message won't be logged because the "
                 "severity level of the logger is set to kInfo.");
   UINFO("This message is logged.");

   // Let's create a simple EventsPrinter that will log all
   // events it will received.
   EventsPrinter p;

   // Register the EventsPrinter to the EventsManager to
   // receive events posted in the application.
   UEventsManager::addHandler(&p);

   // Post some events. As ULogger, UEventsManager
   // is a Singleton and posting events is ThreadSafe.
   UEventsManager::post(new SpecialEvent(1));
   UEventsManager::post(new SpecialEvent(2));
   UEventsManager::post(new SpecialEvent(5));
   UEventsManager::post(new SpecialEvent(7));
   UEventsManager::post(new SpecialEvent(11));

   // Let some times to process the events and quit.
   uSleep(10);

   // Cleanup
   UEventsManager::removeHandler(&p);

   return 0;
}

