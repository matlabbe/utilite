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

#ifndef UVARIANT_H
#define UVARIANT_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines
#include <string>
#include <vector>

/**
 * Experimental class...
 */
class UTILITE_EXP UVariant
{
public:
	enum Type{
		kBool,
		kChar,
		kUChar,
		kShort,
		kUShort,
		kInt,
		kUInt,
		kFloat,
		kDouble,
		kStr,
		kUndef
		};
public:
	UVariant();
	UVariant(const bool & value);
	UVariant(const char & value);
	UVariant(const unsigned char & value);
	UVariant(const short & value);
	UVariant(const unsigned short & value);
	UVariant(const int & value);
	UVariant(const unsigned int & value);
	UVariant(const float & value);
	UVariant(const double & value);
	UVariant(const char * value);
	UVariant(const std::string & value);

	Type type() const {return type_;}

	bool toBool() const;
	char toChar(bool * ok = 0) const;
	unsigned char toUChar(bool * ok = 0) const;
	short toShort(bool * ok = 0) const;
	unsigned short toUShort(bool * ok = 0) const;
	int toInt(bool * ok = 0) const;
	unsigned int toUInt(bool * ok = 0) const;
	float toFloat(bool * ok = 0) const;
	double toDouble(bool * ok = 0) const;
	std::string toStr(bool * ok = 0) const;

	virtual ~UVariant() {}

private:
	Type type_;
	std::vector<unsigned char> data_;
};

#endif /* UVARIANT_H */
