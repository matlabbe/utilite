/**
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UEVENTSMANAGER_H
#define UEVENTSMANAGER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include "utilite/UEventsHandler.h"
#include "utilite/UThreadNode.h"
#include "utilite/ULogger.h"
#include "utilite/UDestroyer.h"

#include <list>

// TODO Not implemented... for multithreading event handling
class UEventDispatcher : public UThreadNode
{
public:
	~UEventDispatcher();
protected:
	friend class UEventsManager;
	UEventDispatcher();

	/**
	 * Method inherited from
	 * StateThread.
	 *
	 * @see StateThread
	 */
	virtual void mainLoop();

private:
	/**
	 * Method inherited from
	 * StateThread.
	 *
	 * @see StateThread
	 */
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
 * To send an event, use EventsManager::post(new Event()).
 * Events are automatically deleted after they are posted.
 *
 * The EventsManager have a list of handlers to which 
 * it sends posted events. To add an handler, use 
 * EventsManager::addHandler(EventsHandler*). To remove, use
 * EventsManager::removeHandler(EventsHandler*).
 *
 * @see post()
 * @see addHandler()
 * @see removeHandler()
 */
class UTILITE_EXP UEventsManager : public UThreadNode{

public:

    /**
     * This method is used to add an events 
     * handler to the list of handlers. It automaticaly 
     * gets the static instance of the EventsManager and calls 
     * his private function _addHandler().
     *
     * @see _addHandler()
     * @param handler the handler to be added.
     */
    static void addHandler(UEventsHandler* handler);

    /**
     * This method is used to remove an events 
     * handler from the list of handlers. It automaticaly 
     * gets the static instance of the EventsManager and calls 
     * his private function _removeHandler().
     *
     * @see _removeHandler()
     * @param handler the handler to be removed.
     */
    static void removeHandler(UEventsHandler* handler);

    /**
     * This method is used to post an event to
     * handlers. It automaticaly gets the static 
     * instance of the EventsManager and calls 
     * his private function _postEvent().
     *
     * @see _postEvent()
     * @param anEvent the event to be posted.
     */
    static void post(UEvent * event, bool async = true);
    
protected:

    /**
     * This method is used to have a reference on the 
     * EventsManager. When no EventsManager exists, one is 
     * created. There is only one instance in the application.
     * See the Singleton pattern further explanation.
     *
     * @return the reference on the EventsManager
     */
    static UEventsManager* getInstance();

    /**
     * Called only once in getInstance(). It can't be instantiated
     * by the user.
     *
     * @see getInstance()
     */
    UEventsManager();

    /**
     * Only called by a Destroyer.
     * @see Destroyer
     */
    virtual ~UEventsManager();

    /**
     * A Destroyer is used to remove a dynamically created
     * Singleton. It is friend here to have access to the 
     * destructor.
     *
     * @see Destroyer
     */
    friend class UDestroyer<UEventsManager>;

    /**
     * Method inherited from 
     * StateThread.
     *
     * @see StateThread
     */
    virtual void mainLoop();

private:

    /**
     * Method inherited from 
     * StateThread.
     *
     * @see StateThread
     */
    virtual void killCleanup();

    /**
     * This method dispatches asynchronised events to all handlers.
     * FIFO (first in first out) dispatching is used.
     */
    virtual void dispatchEvents();

    /**
	 * This method dispatches an event to all handlers.
	 */
    virtual void dispatchEvent(UEvent * event);

    /**
     * This method is used to add an events 
     * handler to the list of handlers.
     *
     * @param handler the handler to be added.
     */
    void _addHandler(UEventsHandler* handler);

    /**
     * This method is used to remove an events 
     * handler from the list of handlers.
     *
     * @param handler the handler to be removed.
     */
    void _removeHandler(UEventsHandler* handler);

    /**
     * This method is used to post an event to
     * handlers.
     *
     * @param anEvent the event to be posted.
     */
    void _postEvent(UEvent * event, bool async = true);
    
private:
    
    static UEventsManager* instance_;            /**< The EventsManager instance pointer. */
    static UDestroyer<UEventsManager> destroyer_; /**< The EventsManager's destroyer. */
    std::list<UEvent*> events_;                /**< The events list. */
    std::list<UEventsHandler*> handlers_;      /**< The handlers list. */
    UMutex eventsMutex_;                         /**< The mutex of the events list, */
    UMutex handlersMutex_;                       /**< The mutex of the handlers list. */
    USemaphore postEventSem_;                    /**< Semaphore used to signal when an events is posted. */
};

#endif // UEVENTSMANAGER_H
