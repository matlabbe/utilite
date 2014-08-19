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

#include "utilite/UThread.h"
#include "utilite/UEventsHandler.h"
#include "utilite/UEventsManager.h"
#include "utilite/UEvent.h"

// Implement a simple event
class ResetEventt : public UEvent
{
public:
	ResetEventt() {}
	virtual ~ResetEventt() {}
	virtual std::string getClassName() const {return "ResetEventt";} // Must be implemented
};

// There is the thread counting indefinitely, the count can be reseted by sending a ResetEvent.
class CounterThread : public UThread, public UEventsHandler
{
public:
	CounterThread() : state_(0), count_(0) {}
	virtual ~CounterThread() {this->join(true);}

protected:
	virtual void mainLoop() {
		if(state_ == 1) {
			state_ = 0;
			// Do a long initialization, reset memory or other special long works... here
			// we reset the count. This could be done in the handleEvent() but
			// with many objects, it is more safe to do it here (in the thread's loop). A safe
			// way could be also to use a UMutex to protect this initialization in
			// the handleEvent(), but it is not recommended to do long works in handleEvent()
			// because this will add latency in the UEventsManager dispatching events loop.
			count_ = 0; // Reset the count
			printf("Reset!\n");
		}

		// Do some works...
		printf("count=%d\n", count_++);
		uSleep(100); // wait 100 ms
	}
	virtual void handleEvent(UEvent * event) {
		if(event->getClassName().compare("ResetEventt") == 0) {
			state_ = 1;
		}
	}
private:
	int state_;
	int count_;
};

int main(int argc, char * argv[])
{
	CounterThread counter;
	counter.start();
	UEventsManager::addHandler(&counter);

	uSleep(500); // wait 500 ms before sending a reset event
	UEventsManager::post(new ResetEventt());
	uSleep(500); // wait 500 ms before termination

	UEventsManager::removeHandler(&counter);
	counter.join(true); // Kill and wait to finish
	return 0;
}
