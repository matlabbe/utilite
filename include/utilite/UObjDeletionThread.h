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

#ifndef UOBJDELETIONTHREAD_H
#define UOBJDELETIONTHREAD_H

#include "utilite/UThreadNode.h"
#include "utilite/UEvent.h"
#include "utilite/UEventsManager.h"

class ObjDeletedEvent : public UEvent
{
public:
	ObjDeletedEvent(int objDeletionThreadId) : UEvent(objDeletionThreadId) {}
	virtual ~ObjDeletedEvent() {}
	virtual std::string getClassName() const {return std::string("ObjDeletedEvent");}
};

template<class T>
class ObjDeletionThread : public UThreadNode
{
public:
	ObjDeletionThread(T * obj, int id=0) :
		obj_(obj),
		id_(id),
		waitMs_(0) {}

	virtual ~ObjDeletionThread()
	{
		join(this);
		if(obj_)
		{
			delete obj_;
		}
	}
	void startDeletion(int waitMs = 0) {waitMs_ = waitMs; this->start();}
	int id() const {return id_;}
	void setObj(T * obj)
	{
		join(this);
		if(obj_)
		{
			delete obj_;
			obj_ = 0;
		}
		obj_ = obj;
	}

private:
	virtual void mainLoop()
	{
		if(waitMs_)
		{
			uSleep(waitMs_);
		}
		if(obj_)
		{
			delete obj_;
			obj_ = 0;
		}
		this->kill();
		UEventsManager::post(new ObjDeletedEvent(id_), false);
	}

private:
	T * obj_;
	int id_;
	int waitMs_;
};

#endif /* UOBJDELETIONTHREAD_H */
