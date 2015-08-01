/*
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

#include "utilite/UVariant.h"

UVariant::UVariant() :
	type_(kUndef)
{
}
UVariant::UVariant(const char & value) :
	type_(kChar),
	data_(sizeof(char))
{
	memcpy(data_.data(), &value, sizeof(char));
}
UVariant::UVariant(const unsigned char & value) :
	type_(kUChar),
	data_(sizeof(unsigned char))
{
	memcpy(data_.data(), &value, sizeof(unsigned char));
}
UVariant::UVariant(const short & value) :
	type_(kShort),
	data_(sizeof(short))
{
	memcpy(data_.data(), &value, sizeof(short));
}
UVariant::UVariant(const unsigned short & value) :
	type_(kUShort),
	data_(sizeof(unsigned short))
{
	memcpy(data_.data(), &value, sizeof(unsigned short));
}
UVariant::UVariant(const int & value) :
	type_(kInt),
	data_(sizeof(int))
{
	memcpy(data_.data(), &value, sizeof(int));
}
UVariant::UVariant(const unsigned int & value) :
	type_(kUInt),
	data_(sizeof(unsigned int))
{
	memcpy(data_.data(), &value, sizeof(unsigned int));
}
UVariant::UVariant(const float & value) :
	type_(kFloat),
	data_(sizeof(float))
{
	memcpy(data_.data(), &value, sizeof(float));
}
UVariant::UVariant(const double & value) :
	type_(kDouble),
	data_(sizeof(double))
{
	memcpy(data_.data(), &value, sizeof(double));
}
UVariant::UVariant(const std::string & value) :
	type_(kStr),
	data_(value.size()+1) // with null character
{
	memcpy(data_.data(), value.data(), value.size()+1);
}

char UVariant::toChar() const
{
	char v = 0;
	if(type_ == kChar)
	{
		memcpy(&v, data_.data(), sizeof(char));
	}
	return v;
}
unsigned char UVariant::toUChar() const
{
	unsigned char v = 0;
	if(type_ == kUChar)
	{
		memcpy(&v, data_.data(), sizeof(unsigned char));
	}
	return v;
}
short UVariant::toShort() const
{
	short v = 0;
	if(type_ == kShort)
	{
		memcpy(&v, data_.data(), sizeof(short));
	}
	return v;
}
unsigned short UVariant::toUShort() const
{
	unsigned short v = 0;
	if(type_ == kUShort)
	{
		memcpy(&v, data_.data(), sizeof(unsigned short));
	}
	return v;
}
int UVariant::toInt() const
{
	int v = 0;
	if(type_ == kInt)
	{
		memcpy(&v, data_.data(), sizeof(int));
	}
	return v;
}
unsigned int UVariant::toUInt() const
{
	unsigned int v = 0;
	if(type_ == kUInt)
	{
		memcpy(&v, data_.data(), sizeof(unsigned int));
	}
	return v;
}
float UVariant::toFloat() const
{
	float v = 0;
	if(type_ == kFloat)
	{
		memcpy(&v, data_.data(), sizeof(float));
	}
	return v;
}
double UVariant::toDouble() const
{
	double v = 0;
	if(type_ == kDouble)
	{
		memcpy(&v, data_.data(), sizeof(double));
	}
	return v;
}
std::string UVariant::toStr() const
{
	std::string v;
	if(type_ == kStr)
	{
		v = std::string((const char *)data_.data());
	}
	return v;
}
