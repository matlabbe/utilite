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

#include <utilite/UThread.h>
#include <utilite/UEventsHandler.h>
#include <utilite/ULogger.h>
#include <utilite/UEvent.h>
#include <utilite/UEventsManager.h>

/**
 * Example of the producer-consumer problem.
 */

// A simple event
class SimpleEvent : public UEvent
{
public:
	SimpleEvent(int level) :
		UEvent(level) {}
	virtual ~SimpleEvent() {}
	virtual std::string getClassName() const {return "SimpleEvent";}
};

// The Producer class
class Producer : public UThread
{
public:
	Producer(const std::string & name, int ms) :
		name_(name),
		rate_(ms) ,
		count_(0) {}

	~Producer() {
		this->join(true);
	}

protected:
	virtual void mainLoop() {
		uSleep(rate_);
		UINFO("%s posting %d", name_.c_str(), count_);
		UEventsManager::post(new SimpleEvent(count_++));
	}

private:
	const std::string name_;
	int rate_;
	int count_;
};

// The Consumer class
class Consumer : public UEventsHandler
{
public:
	Consumer() {}
	~Consumer() {}

protected:
	virtual void handleEvent(UEvent * event) {
		if(event->getClassName().compare("SimpleEvent"))
		{
			UINFO("Consumer received %d", event->getCode());
		}
	}
};

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setPrintWhere(false);

	Consumer c;
	UEventsManager::addHandler(&c);

	Producer p500("p500", 500); //post each 500 ms
	Producer p1000("p1000", 1000); //post each 1000 ms

	// Start the producer threads
	p500.start();
	p1000.start();

	uSleep(3500);
	return 0;
}

