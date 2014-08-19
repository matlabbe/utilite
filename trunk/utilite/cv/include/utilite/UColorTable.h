/*
Copyright (c) 2008-2014, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
