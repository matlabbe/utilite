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

#ifndef UEVENT_H
#define UEVENT_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <string>

class UEventsHandler;

/**
 * This is the base class for all events used 
 * with the EventsManager. Inherited classes 
 * must redefined the virtual method getClassName() 
 * to return their class name.
 *
 * @see EventsManager
 * @see getClassName()
 */
class UTILITE_EXP UEvent{
public:
    /**
     * @param code the event code.
     * TODO : Remove the code, not required for most of all implemented events
     */
    UEvent(int code = 0) : code_(code) {}

    virtual ~UEvent() {}

    /**
     * This method is used to get the class name 
     * of the event. For example, if the class MouseEvent 
     * inherits from Event, it must return the
     * "MouseEvent" string.
     * @return std::string the class name
     */
    virtual std::string getClassName() const {return "UEvent";} // TODO : macro?

    int getCode() const {return code_;}

protected:

private:
    int code_; /**< The event's code. */
};

#endif // UEVENT_H
