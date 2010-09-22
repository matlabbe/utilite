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

#ifndef OBJDELETIONTHREAD_H_
#define OBJDELETIONTHREAD_H_

#include "UStateThread.h"
#include "UEvent.h"
#include "UEventsManager.h"

class ObjDeletedEvent : public UEvent
{
public:
	ObjDeletedEvent(int objDeletionThreadId) : UEvent(objDeletionThreadId) {}
	virtual ~ObjDeletedEvent() {}
	virtual std::string getClassName() const {return std::string("ObjDeletedEvent");}
};

template<class T>
class ObjDeletionThread : public UStateThread
{
public:
	ObjDeletionThread(T * obj, int id=0) :
		_obj(obj),
		_id(id),
		_waitMs(0) {}

	virtual ~ObjDeletionThread()
	{
		this->killSafely();
		if(_obj)
		{
			delete _obj;
		}
	}
	void startDeletion(int waitMs = 0) {_waitMs = waitMs; this->startThread();}
	int id() const {return _id;}
	void setObj(T * obj)
	{
		this->killSafely();
		if(_obj)
		{
			delete _obj;
			_obj = 0;
		}
		_obj = obj;
	}

private:
	virtual void threadInnerLoop()
	{
		if(_waitMs)
		{
			SLEEP(_waitMs);
		}
		if(_obj)
		{
			delete _obj;
			_obj = 0;
		}
		this->killSafely();
		UEventsManager::postEvent(new ObjDeletedEvent(_id), false);
	}

private:
	T * _obj;
	int _id;
	int _waitMs;
};

#endif /* OBJDELETIONTHREAD_H_ */
