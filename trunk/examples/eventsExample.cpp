#include <UEventsManager.h>
#include <UEventsHandler.h>
#include <UEvent.h>
#include <ULogger.h>

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
         ULOGGER_INFO("SpecialEvent \"%d\" received!", e->getCode());
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
   ULOGGER_DEBUG("This message won't be logged because the "
                 "severity level of the logger is set to kInfo.");
   ULOGGER_INFO("This message is logged.");

   // Lets create a simple EventsPrinter that will log all
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

