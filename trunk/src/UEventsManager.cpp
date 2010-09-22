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

#include "UEventsManager.h"
#include "ULogger.h"
#include "UEvent.h"
#include <list>
#include "UStl.h"

UEventsManager* UEventsManager::_instance = 0;
UDestroyer<UEventsManager> UEventsManager::_destroyer;

void UEventsManager::addHandler(UEventsHandler* handler)
{
    UEventsManager::getInstance()->_addHandler(handler);
}

void UEventsManager::removeHandler(UEventsHandler* handler)
{
    UEventsManager::getInstance()->_removeHandler(handler);
}

void UEventsManager::postEvent(UEvent * event, bool async)
{
    UEventsManager::getInstance()->_postEvent(event, async);
}

UEventsManager* UEventsManager::getInstance()
{
    if(!_instance)
    {
        _instance = new UEventsManager();
        _destroyer.setDoomed(_instance);
        _instance->startThread(); // Start the thread
    }
    return _instance;
}

UEventsManager::UEventsManager()
{
}

UEventsManager::~UEventsManager()
{
	LOGGER_DEBUG("");
    this->killSafely();

    // Free memory
    for(std::list<UEvent*>::iterator it=_events.begin(); it!=_events.end(); ++it)
    {
        delete *it;
    }
    _events.clear();

    _handlers.clear();

    _instance = 0;

    //printf("EventsManager is destroyed...\n\r");
}

void UEventsManager::threadInnerLoop()
{
    _postEventSem.acquire();
    if(!this->isKilled())
    {
		dispatchEvents();
    }
}

void UEventsManager::killSafelyInner()
{
    _postEventSem.release();
}

void UEventsManager::dispatchEvents()
{
    if(_events.size() == 0)
    {
        return;
    }

    std::list<UEvent*>::iterator it;
    std::list<UEvent*> eventsBuf;

    // Copy events in a buffer :
    // Other threads can post events 
    // while events are handled.
    _eventsMutex.Lock();
    {
        eventsBuf = _events;
        _events.clear();
    }
    _eventsMutex.Unlock();

	// Past events to handlers
	for(it=eventsBuf.begin(); it!=eventsBuf.end(); ++it)
	{
		dispatchEvent(*it);
		delete *it;
	}
    eventsBuf.clear();
}

void UEventsManager::dispatchEvent(UEvent * event)
{
	UEventsHandler * handler;
	_handlersMutex.Lock();
	for(std::list<UEventsHandler*>::iterator it=_handlers.begin(); it!=_handlers.end(); ++it)
	{
		handler = *it;
		_handlersMutex.Unlock();

		// To be able to add/remove an handler in a handleEvent call (without a deadlock)
		// @see _addHandler(), _removeHandler()
		handler->handleEvent(event);

		_handlersMutex.Lock();
	}
	_handlersMutex.Unlock();

}

void UEventsManager::_addHandler(UEventsHandler* handler)
{
    if(!this->isKilled())
    {
        _handlersMutex.Lock();
        {
        	//make sure it is not already in the list
        	bool handlerFound = false;
        	for(std::list<UEventsHandler*>::iterator it=_handlers.begin(); it!=_handlers.end(); ++it)
        	{
        		if(*it == handler)
        		{
        			handlerFound = true;
        		}
        	}
        	if(!handlerFound)
        	{
        		_handlers.push_back(handler);
        	}
        }
        _handlersMutex.Unlock();
    }
}

void UEventsManager::_removeHandler(UEventsHandler* handler)
{
    if(!this->isKilled())
    {
        _handlersMutex.Lock();
        {
            for (std::list<UEventsHandler*>::iterator it = _handlers.begin(); it!=_handlers.end(); ++it)
            {
                if(*it == handler)
                {
                    _handlers.erase(it);
                    break;
                }
            }
        }
        _handlersMutex.Unlock();
    }
}

void UEventsManager::_postEvent(UEvent * event, bool async)
{
    if(!this->isKilled())
    {
    	if(async)
    	{
			_eventsMutex.Lock();
			{
				_events.push_back(event);
			}
			_eventsMutex.Unlock();

			// Signal the EventsManager that an Event is added
			_postEventSem.release();
    	}
    	else
    	{
    		dispatchEvent(event);
    		delete event;
    	}
    }
    else
    {
    	delete event;
    }
}
