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

#ifndef UEVENTSMANAGER_H
#define UEVENTSMANAGER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include "utilite/UEventsHandler.h"
#include "utilite/UThreadNode.h"
#include "utilite/ULogger.h"
#include "utilite/UDestroyer.h"

#include <list>
#include <map>

// TODO Not implemented... for multithreading event handling
class UEventDispatcher : public UThread
{
public:
	virtual ~UEventDispatcher();
protected:
	friend class UEventsManager;
	UEventDispatcher();

	virtual void mainLoop();

private:
	virtual void killCleanup();

private:
	UEvent * _event;
	std::vector<UEventsHandler*> _handlers;
};

/**
 * This class is used to post events between threads 
 * in the application. It is Thread-Safe and the events are sent 
 * to receivers in the same order they are posted (FIFO). It works 
 * like the design pattern Mediator. It is also a Singleton, so 
 * it can be used anywhere in the application. 
 *
 * To send an event, use UEventsManager::post().
 * Events are automatically deleted after they are posted.
 *
 * The EventsManager have a list of handlers to which 
 * it sends posted events. To add an handler, use 
 * UEventsManager::addHandler(). To remove, use
 * UEventsManager::removeHandler().
 *
 * @code
 *  // Anywhere in the code:
 *  UEventsManager::post(new MyEvent()); // where MyEvent is an implemented UEvent
 * @endcode
 *
 * @see UEvent
 * @see UEventsHandler
 * @see post()
 * @see addHandler()
 * @see removeHandler()
 */
class UTILITE_EXP UEventsManager : public UThread{

public:

    /**
     * This method is used to add an events 
     * handler to the list of handlers.
     *
     * @param handler the handler to be added.
     */
    static void addHandler(UEventsHandler* handler);

    /**
     * This method is used to remove an events 
     * handler from the list of handlers.
     *
     * @param handler the handler to be removed.
     */
    static void removeHandler(UEventsHandler* handler);

    /**
     * This method is used to post an event to
     * handlers.
     *
     * Event can be posted asynchronously or not. In the first case,
     * the event is dispatched by the UEventsManager's thread. In the
     * second case, the event is handled immediately by event's
     * receivers, thus in the sender thread.
     *
     * @param event the event to be posted.
     * @param async if true, the event is dispatched by the UEventsManager thread, otherwise it's in the caller thread (synchronous).
     */
    static void post(UEvent * event, bool async = true, const UEventsSender * sender = 0);

    static void createPipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName);

    static void removePipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName);

    static void removeAllPipes(const UEventsSender * sender);
    static void removeNullPipes(const UEventsSender * sender);

protected:

    /*
     * This method is used to have a reference on the 
     * EventsManager. When no EventsManager exists, one is 
     * created. There is only one instance in the application.
     * See the Singleton pattern further explanation.
     *
     * @return the reference on the EventsManager
     */
    static UEventsManager* getInstance();

    /*
     * Called only once in getInstance(). It can't be instantiated
     * by the user.
     *
     */
    UEventsManager();

    /*
     * Only called by a Destroyer.
     */
    virtual ~UEventsManager();

    /*
     * A Destroyer is used to remove a dynamically created
     * Singleton. It is friend here to have access to the 
     * destructor.
     *
     */
    friend class UDestroyer<UEventsManager>;

    /**
	 * The UEventsManager's main loop.
	 */
    virtual void mainLoop();

private:

    /**
	 * Reimplemented to wake up UEventsManager on termination.
	 */
    virtual void mainLoopKill();

    /*
     * This method dispatches asynchronized events to all handlers.
     * FIFO (first in first out) dispatching is used.
     */
    virtual void dispatchEvents();

    /*
	 * This method dispatches an event to all handlers.
	 */
    virtual void dispatchEvent(UEvent * event, const UEventsSender * sender);

    /*
     * This method is used to add an events 
     * handler to the list of handlers.
     *
     * @param handler the handler to be added.
     */
    void _addHandler(UEventsHandler* handler);

    /*
     * This method is used to remove an events 
     * handler from the list of handlers.
     *
     * @param handler the handler to be removed.
     */
    void _removeHandler(UEventsHandler* handler);

    /*
     * This method is used to post an event to
     * handlers.
     *
     * Event can be posted asynchronously or not. In the first case,
     * the event is dispatched by the UEventsManager's thread. In the
     * second case, the event is handled immediately by event's
     * receivers, thus in the sender thread.
     *
     * @param event the event to be posted.
     * @param async if true, the event is dispatched by the UEventsManager thread, otherwise it's in the caller thread (synchronous).
     */
    void _postEvent(UEvent * event, bool async = true, const UEventsSender * sender = 0);

    std::list<UEventsHandler*> getPipes(
    		const UEventsSender * sender,
    		const std::string & eventName);

    void _createPipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName);

    void _removePipe(
		const UEventsSender * sender,
		const UEventsHandler * receiver,
		const std::string & eventName);

    void _removeAllPipes(const UEventsSender * sender);
    void _removeNullPipes(const UEventsSender * sender);

private:
    
    class Pipe
    {
    public:
    	Pipe(const UEventsSender * sender, const UEventsHandler * receiver, const std::string & eventName) :
    		sender_(sender),
    		receiver_(receiver),
    		eventName_(eventName)
    	{}
    	const UEventsSender * sender_;
    	const UEventsHandler * receiver_;
    	const std::string eventName_;
    };

    static UEventsManager* instance_;            /* The EventsManager instance pointer. */
    static UDestroyer<UEventsManager> destroyer_; /* The EventsManager's destroyer. */
    std::list<std::pair<UEvent*, const UEventsSender * > > events_; /* The events list. */
    std::list<UEventsHandler*> handlers_;      /* The handlers list. */
    UMutex eventsMutex_;                         /* The mutex of the events list, */
    UMutex handlersMutex_;                       /* The mutex of the handlers list. */
    USemaphore postEventSem_;                    /* Semaphore used to signal when an events is posted. */
    std::list<Pipe> pipes_;
    UMutex pipesMutex_;
};

#endif // UEVENTSMANAGER_H
