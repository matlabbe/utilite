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

#ifndef COLORTABLE_H
#define COLORTABLE_H

#include "utilite/UtiLiteCvExp.h" // DLL export/import defines

#include <vector>

/**
 * A color table.
 * Note INDEXED_TABLE_# are generated using MATLAB with command "ceil(colorcube(#)*255)".
 */
//
class UTILITECV_EXP UColorTable
{
public:
	enum Size{kSize8 = 8,
			kSize16 = 16,
			kSize32 = 32,
			kSize64 = 64,
			kSize128 = 128,
			kSize256 = 256,
			kSize512 = 512,
			kSize1024 = 1024,
			kSize2048 = 2048,
			kSize4096 = 4096,
			kSize8192 = 8192,
			kSize16384 = 16384,
			kSize32768 = 32768,
			kSize65536 = 65536,
			kSize16777216 = 16777216};
public:
	UColorTable(int size);
	virtual ~UColorTable() {}

	static unsigned char INDEXED_TABLE_8[24];
	static unsigned char INDEXED_TABLE_16[48];
	static unsigned char INDEXED_TABLE_32[96];
	static unsigned char INDEXED_TABLE_64[192];
	static unsigned char INDEXED_TABLE_128[384];
	static unsigned char INDEXED_TABLE_256[768];
	static unsigned char INDEXED_TABLE_512[1536];
	static unsigned char INDEXED_TABLE_1024[3076];
	static unsigned char INDEXED_TABLE_2048[6144];
	static unsigned char INDEXED_TABLE_4096[12288];
	static unsigned char INDEXED_TABLE_8192[24576];
	static unsigned char INDEXED_TABLE_16384[49152];
	static unsigned char INDEXED_TABLE_32768[98304];
	static unsigned char INDEXED_TABLE_65536[196608];

	int size() const {return _size;}
	unsigned int getIndex(unsigned char r, unsigned char g, unsigned char b) const;
	void getRgb(unsigned int index, unsigned char & r, unsigned char & g, unsigned char & b) const;

private:
	int _size;
	std::vector<unsigned short> _rgb2indexed;
	unsigned char * _indexedTable;
};

#endif // COLORTABLE_H
