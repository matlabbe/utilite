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

#ifndef VARIANT_H_
#define VARIANT_H_

#include <string>

class UVariant
{
public:
	virtual ~UVariant() {}

	virtual std::string className() const = 0;

	template<class T>
	const T data() const {
		if(_data)
			return (T)_data;
		return (const T)_constData;
	}

	template<class T>
	T takeDataOwnership() {
		T data = (T)_data;
		_constData = 0;
		_data=0;
		return data;
	}

protected:
	UVariant(void * data) :
		_data(data),
		_constData(0)
	{}

	UVariant(const void * data) :
		_data(0),
		_constData(data)
	{}

protected:
	void * _data;
	const void * _constData;
};

#endif /* VARIANT_H_ */
