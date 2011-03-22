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

#ifndef UEVENTSHANDLER_H
#define UEVENTSHANDLER_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

class UEvent;

 /**
  * This class is a base abstract class for 
  * handling events.
  * @see EventsManager
  * @see Event
  *
  */
class UTILITE_EXP UEventsHandler{
public:
    

protected:
    /**
     * Only the EventsManager has access 
     * to the handleEvent method.
     */
    friend class UEventsManager;

    /**
     * Method called by the EventsManager 
     * to handle an event. Important : this method 
     * must do a minimum of work because the faster 
     * the dispatching loop is done; the faster the 
     * events are received. If a handling function 
     * takes too much time, the events list can grow 
     * faster than it is emptied. The event can be
     * modified but must not be deleted.
     *
     */
    virtual void handleEvent(UEvent * anEvent) = 0;

protected:
    /**
     * EventsHandler constructor.
     *
     * Note : You can call EventsManager::addHandler(this) at
     * the end of the constructor of the inherited class where the virtual
     * method handleEvent(...) is defined. If so, the EventsHandler doesn't
     * need to be manually added to the EventsManager where the handler
     * is instantiated. We decided to not include EventsManager::addHandler(this)
     * in this abstract class constructor because an event can be handled (calling
     * the pure virtual method) while the concrete class is constructed.
     */
    UEventsHandler() {}

    /**
     * EventsHandler destuctor.
     *
     * By default, it removes the handler reference from the EventsManager. To be thread-safe,
     * the inherited class must remove itself from the EventsManager before it is deleted because
     * an event can be handled (calling the pure virtual method handleEvent()) after the concrete class
     * is deleted.
     */
    virtual ~UEventsHandler();

    /**
     * Post an event. Same has calling UEventsManager::post();
     */
    void post(UEvent * event, bool async = true);

private:
    
};

#endif // UEVENTSHANDLER_H
