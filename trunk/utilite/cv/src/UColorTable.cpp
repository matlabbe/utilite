/*
 * ColorTable.cpp
 *
 *  Created on: Nov 26, 2011
 *,  Author: MatLab
 */

#include "utilite/UColorTable.h"
#include "ColorIndexes65536_bin_zip.h"
#include "ColorIndexes32768_bin_zip.h"
#include "ColorIndexes16384_bin_zip.h"
#include "ColorIndexes8192_bin_zip.h"
#include "ColorIndexes4096_bin_zip.h"
#include "ColorIndexes2048_bin_zip.h"
#include "ColorIndexes1024_bin_zip.h"
#include "ColorIndexes512_bin_zip.h"
#include "ColorIndexes256_bin_zip.h"
#include "ColorIndexes128_bin_zip.h"
#include "ColorIndexes64_bin_zip.h"
#include "ColorIndexes32_bin_zip.h"
#include "ColorIndexes16_bin_zip.h"
#include "ColorIndexes8_bin_zip.h"
#include <zlib.h>
#include <string.h>
#include <utilite/ULogger.h>
#include <utilite/UConversion.h>

namespace rtabmap
{
UColorTable::UColorTable(int size) :
	_size(size),
	_rgb2indexed(0)
{
	std::string hex;
	switch(size)
	{
	case kSize8:
		hex = COLORINDEXES8_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_8;
		break;
	case kSize16:
		hex = COLORINDEXES16_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_16;
		break;
	case kSize32:
		hex = COLORINDEXES32_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_32;
		break;
	case kSize64:
		hex = COLORINDEXES64_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_64;
		break;
	case kSize128:
		hex = COLORINDEXES128_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_128;
		break;
	case kSize256:
		hex = COLORINDEXES256_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_256;
		break;
	case kSize512:
		hex = COLORINDEXES512_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_512;
		break;
	case kSize1024:
		hex = COLORINDEXES1024_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_1024;
		break;
	case kSize2048:
		hex = COLORINDEXES2048_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_2048;
		break;
	case kSize4096:
		hex = COLORINDEXES4096_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_4096;
		break;
	case kSize8192:
		hex = COLORINDEXES8192_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_8192;
		break;
	case kSize16384:
		hex = COLORINDEXES16384_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_16384;
		break;
	case kSize32768:
		hex = COLORINDEXES32768_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_32768;
		break;
	case kSize65536:
		hex = COLORINDEXES65536_BIN_ZIP;
		_indexedTable = INDEXED_TABLE_65536;
		break;
	case kSize16777216:
		_indexedTable = 0;
		break;
	default:
		UFATAL("Undefined size %d", size);
		break;
	}

	if(size != kSize16777216)
	{
		// Uncompress the table
		std::vector<char> bytes = uHex2Bytes(hex);
		std::vector<unsigned char> uncompressed(256*256*256*sizeof(unsigned short));
		uLongf totalUncompressed = uncompressed.size();
		UDEBUG("zip bytes=%d, uncompressed prediction=%d", (int)bytes.size(), (int)totalUncompressed);
		int err_code = uncompress((Bytef*)uncompressed.data(), &totalUncompressed, (const Bytef*)bytes.data(), bytes.size());
		UDEBUG("totalUncompressed=%ld", totalUncompressed);

		if(err_code == Z_MEM_ERROR)
		{
			UFATAL("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(err_code == Z_BUF_ERROR)
		{
			UFATAL("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
		else if(err_code == Z_DATA_ERROR)
		{
			UFATAL("Z_DATA_ERROR : The compressed data (referenced by source) was corrupted.");
		}

		_rgb2indexed = std::vector<unsigned short>(uncompressed.size()/sizeof(unsigned short));
		memcpy(_rgb2indexed.data(), uncompressed.data(), uncompressed.size());
		//for(unsigned int i=0; i<_rgb2indexed.size();++i)
		//{
		//	UDEBUG("%d=%d",i, _rgb2indexed[i]);
		//}
		UDEBUG("_rgb2indexed=%d, bytes=%d", _rgb2indexed.size(), uncompressed.size());
	}
}

unsigned int UColorTable::getIndex(unsigned char r, unsigned char g, unsigned char b) const
{
	if(_size != kSize16777216)
	{
		return _rgb2indexed[int(b)*256*256 + int(g)*256 + int(r)];
	}
	else
	{
		return int(b)*256*256 + int(g)*256 + int(r);
	}
}

void UColorTable::getRgb(unsigned int index, unsigned char & r, unsigned char & g, unsigned char & b) const
{
	if(_size != kSize16777216)
	{
		if(index > (unsigned int)this->size()*3)
		{
			UFATAL("Color index out of bounds (%d>%d)", (int)index, this->size()*3);
		}
		b = _indexedTable[index*3+2];
		g = _indexedTable[index*3+1];
		r = _indexedTable[index*3];
	}
	else
	{
		r = index & 0xFFFF;
		g = (index >> 8) & 0xFFFF;
		b = (index >> 16) & 0xFFFF;
	}
}

unsigned char UColorTable::INDEXED_TABLE_8[24] = {
  255, 255, 0
, 0, 255, 255
, 255, 0, 255
, 255, 0, 0
, 0, 255, 0
, 0, 0, 255
, 0, 0, 0
, 255, 255, 255
};

unsigned char UColorTable::INDEXED_TABLE_16[48] = {
  255, 255, 0
, 0, 255, 255
, 255, 0, 255
, 85, 0, 0
, 170, 0, 0
, 255, 0, 0
, 0,  85, 0
, 0, 170, 0
, 0, 255, 0
, 0, 0,  85
, 0, 0, 170
, 0, 0, 255
, 0, 0, 0
, 85,  85,  85
, 170, 170, 170
, 255, 255, 255
};

unsigned char UColorTable::INDEXED_TABLE_32[96] = {
  128, 128, 0
, 128, 255, 0
, 255, 128, 0
, 255, 255, 0
, 0, 128, 128
, 0, 255, 128
, 128, 0, 128
, 128, 255, 128
, 255, 0, 128
, 255, 128, 128
, 255, 255, 128
, 0, 128, 255
, 0, 255, 255
, 128, 0, 255
, 128, 128, 255
, 128, 255, 255
, 255, 0, 255
, 255, 128, 255
, 85, 0, 0
, 170, 0, 0
, 255, 0, 0
, 0, 85, 0
, 0, 170, 0
, 0, 255, 0
, 0, 0, 85
, 0, 0, 170
, 0, 0, 255
, 0, 0, 0
, 64, 64, 64
, 128, 128, 128
, 191, 191, 191
, 255, 255, 255
};

unsigned char UColorTable::INDEXED_TABLE_64[192] = {
  85, 85, 0
, 85, 170, 0
, 85, 255, 0
, 170, 85, 0
, 170, 170, 0
, 170, 255, 0
, 255, 85, 0
, 255, 170, 0
, 255, 255, 0
, 0, 85, 128
, 0, 170, 128
, 0, 255, 128
, 85, 0, 128
, 85, 85, 128
, 85, 170, 128
, 85, 255, 128
, 170, 0, 128
, 170, 85, 128
, 170, 170, 128
, 170, 255, 128
, 255, 0, 128
, 255, 85, 128
, 255, 170, 128
, 255, 255, 128
, 0, 85, 255
, 0, 170, 255
, 0, 255, 255
, 85, 0, 255
, 85, 85, 255
, 85, 170, 255
, 85, 255, 255
, 170, 0, 255
, 170, 85, 255
, 170, 170, 255
, 170, 255, 255
, 255, 0, 255
, 255, 85, 255
, 255, 170, 255
, 43, 0, 0
, 85, 0, 0
, 128, 0, 0
, 170, 0, 0
, 213, 0, 0
, 255, 0, 0
, 0, 43, 0
, 0, 85, 0
, 0, 128, 0
, 0, 170, 0
, 0, 213, 0
, 0, 255, 0
, 0, 0, 43
, 0, 0, 85
, 0, 0, 128
, 0, 0, 170
, 0, 0, 213
, 0, 0, 255
, 0, 0, 0
, 36, 36, 36
, 73, 73, 73
, 109, 109, 109
, 146, 146, 146
, 182, 182, 182
, 219, 219, 219
, 255, 255, 255
};

unsigned char UColorTable::INDEXED_TABLE_128[384] = {
  64, 64,  0
, 64, 128,  0
, 64, 191,  0
, 64, 255,  0
, 128, 64,  0
, 128, 128,  0
, 128, 191,  0
, 128, 255,  0
, 191, 64,  0
, 191, 128,  0
, 191, 191,  0
, 191, 255,  0
, 255, 64,  0
, 255, 128,  0
, 255, 191,  0
, 255, 255,  0
, 0, 64, 64
, 0, 128, 64
, 0, 191, 64
, 0, 255, 64
, 64,  0, 64
, 64, 128, 64
, 64, 191, 64
, 64, 255, 64
, 128,  0, 64
, 128, 64, 64
, 128, 128, 64
, 128, 191, 64
, 128, 255, 64
, 191,  0, 64
, 191, 64, 64
, 191, 128, 64
, 191, 191, 64
, 191, 255, 64
, 255,  0, 64
, 255, 64, 64
, 255, 128, 64
, 255, 191, 64
, 255, 255, 64
, 0, 64, 128
, 0, 128, 128
, 0, 191, 128
, 0, 255, 128
, 64,  0, 128
, 64, 64, 128
, 64, 128, 128
, 64, 191, 128
, 64, 255, 128
, 128,  0, 128
, 128, 64, 128
, 128, 191, 128
, 128, 255, 128
, 191,  0, 128
, 191, 64, 128
, 191, 128, 128
, 191, 191, 128
, 191, 255, 128
, 255,  0, 128
, 255, 64, 128
, 255, 128, 128
, 255, 191, 128
, 255, 255, 128
, 0, 64, 191
, 0, 128, 191
, 0, 191, 191
, 0, 255, 191
, 64,  0, 191
, 64, 64, 191
, 64, 128, 191
, 64, 191, 191
, 64, 255, 191
, 128,  0, 191
, 128, 64, 191
, 128, 128, 191
, 128, 191, 191
, 128, 255, 191
, 191,  0, 191
, 191, 64, 191
, 191, 128, 191
, 191, 255, 191
, 255,  0, 191
, 255, 64, 191
, 255, 128, 191
, 255, 191, 191
, 255, 255, 191
, 0, 64, 255
, 0, 128, 255
, 0, 191, 255
, 0, 255, 255
, 64,  0, 255
, 64, 64, 255
, 64, 128, 255
, 64, 191, 255
, 64, 255, 255
, 128,  0, 255
, 128, 64, 255
, 128, 128, 255
, 128, 191, 255
, 128, 255, 255
, 191,  0, 255
, 191, 64, 255
, 191, 128, 255
, 191, 191, 255
, 191, 255, 255
, 255,  0, 255
, 255, 64, 255
, 255, 128, 255
, 255, 191, 255
, 64,  0,  0
, 128,  0,  0
, 191,  0,  0
, 255,  0,  0
, 0, 64,  0
, 0, 128,  0
, 0, 191,  0
, 0, 255,  0
, 0,  0, 64
, 0,  0, 128
, 0,  0, 191
, 0,  0, 255
, 0,  0,  0
, 36, 36, 36
, 73, 73, 73
, 109, 109, 109
, 146, 146, 146
, 182, 182, 182
, 219, 219, 219
, 255, 255, 255
};

unsigned char UColorTable::INDEXED_TABLE_256[768] = {
  51, 51, 0
, 51, 102, 0
, 51, 153, 0
, 51, 204, 0
, 51, 255, 0
, 102, 51, 0
, 102, 102, 0
, 102, 153, 0
, 102, 204, 0
, 102, 255, 0
, 153, 51, 0
, 153, 102, 0
, 153, 153, 0
, 153, 204, 0
, 153, 255, 0
, 204, 51, 0
, 204, 102, 0
, 204, 153, 0
, 204, 204, 0
, 204, 255, 0
, 255, 51, 0
, 255, 102, 0
, 255, 153, 0
, 255, 204, 0
, 255, 255, 0
, 0, 51, 51
, 0, 102, 51
, 0, 153, 51
, 0, 204, 51
, 0, 255, 51
, 51, 0, 51
, 51, 102, 51
, 51, 153, 51
, 51, 204, 51
, 51, 255, 51
, 102, 0, 51
, 102, 51, 51
, 102, 102, 51
, 102, 153, 51
, 102, 204, 51
, 102, 255, 51
, 153, 0, 51
, 153, 51, 51
, 153, 102, 51
, 153, 153, 51
, 153, 204, 51
, 153, 255, 51
, 204, 0, 51
, 204, 51, 51
, 204, 102, 51
, 204, 153, 51
, 204, 204, 51
, 204, 255, 51
, 255, 0, 51
, 255, 51, 51
, 255, 102, 51
, 255, 153, 51
, 255, 204, 51
, 255, 255, 51
, 0, 51, 102
, 0, 102, 102
, 0, 153, 102
, 0, 204, 102
, 0, 255, 102
, 51, 0, 102
, 51, 51, 102
, 51, 102, 102
, 51, 153, 102
, 51, 204, 102
, 51, 255, 102
, 102, 0, 102
, 102, 51, 102
, 102, 153, 102
, 102, 204, 102
, 102, 255, 102
, 153, 0, 102
, 153, 51, 102
, 153, 102, 102
, 153, 153, 102
, 153, 204, 102
, 153, 255, 102
, 204, 0, 102
, 204, 51, 102
, 204, 102, 102
, 204, 153, 102
, 204, 204, 102
, 204, 255, 102
, 255, 0, 102
, 255, 51, 102
, 255, 102, 102
, 255, 153, 102
, 255, 204, 102
, 255, 255, 102
, 0, 51, 153
, 0, 102, 153
, 0, 153, 153
, 0, 204, 153
, 0, 255, 153
, 51, 0, 153
, 51, 51, 153
, 51, 102, 153
, 51, 153, 153
, 51, 204, 153
, 51, 255, 153
, 102, 0, 153
, 102, 51, 153
, 102, 102, 153
, 102, 153, 153
, 102, 204, 153
, 102, 255, 153
, 153, 0, 153
, 153, 51, 153
, 153, 102, 153
, 153, 204, 153
, 153, 255, 153
, 204, 0, 153
, 204, 51, 153
, 204, 102, 153
, 204, 153, 153
, 204, 204, 153
, 204, 255, 153
, 255, 0, 153
, 255, 51, 153
, 255, 102, 153
, 255, 153, 153
, 255, 204, 153
, 255, 255, 153
, 0, 51, 204
, 0, 102, 204
, 0, 153, 204
, 0, 204, 204
, 0, 255, 204
, 51, 0, 204
, 51, 51, 204
, 51, 102, 204
, 51, 153, 204
, 51, 204, 204
, 51, 255, 204
, 102, 0, 204
, 102, 51, 204
, 102, 102, 204
, 102, 153, 204
, 102, 204, 204
, 102, 255, 204
, 153, 0, 204
, 153, 51, 204
, 153, 102, 204
, 153, 153, 204
, 153, 204, 204
, 153, 255, 204
, 204, 0, 204
, 204, 51, 204
, 204, 102, 204
, 204, 153, 204
, 204, 255, 204
, 255, 0, 204
, 255, 51, 204
, 255, 102, 204
, 255, 153, 204
, 255, 204, 204
, 255, 255, 204
, 0, 51, 255
, 0, 102, 255
, 0, 153, 255
, 0, 204, 255
, 0, 255, 255
, 51, 0, 255
, 51, 51, 255
, 51, 102, 255
, 51, 153, 255
, 51, 204, 255
, 51, 255, 255
, 102, 0, 255
, 102, 51, 255
, 102, 102, 255
, 102, 153, 255
, 102, 204, 255
, 102, 255, 255
, 153, 0, 255
, 153, 51, 255
, 153, 102, 255
, 153, 153, 255
, 153, 204, 255
, 153, 255, 255
, 204, 0, 255
, 204, 51, 255
, 204, 102, 255
, 204, 153, 255
, 204, 204, 255
, 204, 255, 255
, 255, 0, 255
, 255, 51, 255
, 255, 102, 255
, 255, 153, 255
, 255, 204, 255
, 17, 0, 0
, 34, 0, 0
, 51, 0, 0
, 68, 0, 0
, 85, 0, 0
, 102, 0, 0
, 119, 0, 0
, 136, 0, 0
, 153, 0, 0
, 170, 0, 0
, 187, 0, 0
, 204, 0, 0
, 221, 0, 0
, 238, 0, 0
, 255, 0, 0
, 0, 17, 0
, 0, 34, 0
, 0, 51, 0
, 0, 68, 0
, 0, 85, 0
, 0, 102, 0
, 0, 119, 0
, 0, 136, 0
, 0, 153, 0
, 0, 170, 0
, 0, 187, 0
, 0, 204, 0
, 0, 221, 0
, 0, 238, 0
, 0, 255, 0
, 0, 0, 17
, 0, 0, 34
, 0, 0, 51
, 0, 0, 68
, 0, 0, 85
, 0, 0, 102
, 0, 0, 119
, 0, 0, 136
, 0, 0, 153
, 0, 0, 170
, 0, 0, 187
, 0, 0, 204
, 0, 0, 221
, 0, 0, 238
, 0, 0, 255
, 0, 0, 0
, 17, 17, 17
, 34, 34, 34
, 51, 51, 51
, 68, 68, 68
, 85, 85, 85
, 102, 102, 102
, 119, 119, 119
, 136, 136, 136
, 153, 153, 153
, 170, 170, 170
, 187, 187, 187
, 204, 204, 204
, 221, 221, 221
, 238, 238, 238
, 255, 255, 255
};

unsigned char UColorTable::INDEXED_TABLE_512[1536] = {
  43,  43, 0
,  43,  85, 0
,  43, 128, 0
,  43, 170, 0
,  43, 213, 0
,  43, 255, 0
,  85,  43, 0
,  85,  85, 0
,  85, 128, 0
,  85, 170, 0
,  85, 213, 0
,  85, 255, 0
, 128,  43, 0
, 128,  85, 0
, 128, 128, 0
, 128, 170, 0
, 128, 213, 0
, 128, 255, 0
, 170,  43, 0
, 170,  85, 0
, 170, 128, 0
, 170, 170, 0
, 170, 213, 0
, 170, 255, 0
, 213,  43, 0
, 213,  85, 0
, 213, 128, 0
, 213, 170, 0
, 213, 213, 0
, 213, 255, 0
, 255,  43, 0
, 255,  85, 0
, 255, 128, 0
, 255, 170, 0
, 255, 213, 0
, 255, 255, 0
, 0,  43,  43
, 0,  85,  43
, 0, 128,  43
, 0, 170,  43
, 0, 213,  43
, 0, 255,  43
,  43, 0,  43
,  43,  85,  43
,  43, 128,  43
,  43, 170,  43
,  43, 213,  43
,  43, 255,  43
,  85, 0,  43
,  85,  43,  43
,  85,  85,  43
,  85, 128,  43
,  85, 170,  43
,  85, 213,  43
,  85, 255,  43
, 128, 0,  43
, 128,  43,  43
, 128,  85,  43
, 128, 128,  43
, 128, 170,  43
, 128, 213,  43
, 128, 255,  43
, 170, 0,  43
, 170,  43,  43
, 170,  85,  43
, 170, 128,  43
, 170, 170,  43
, 170, 213,  43
, 170, 255,  43
, 213, 0,  43
, 213,  43,  43
, 213,  85,  43
, 213, 128,  43
, 213, 170,  43
, 213, 213,  43
, 213, 255,  43
, 255, 0,  43
, 255,  43,  43
, 255,  85,  43
, 255, 128,  43
, 255, 170,  43
, 255, 213,  43
, 255, 255,  43
, 0,  43,  85
, 0,  85,  85
, 0, 128,  85
, 0, 170,  85
, 0, 213,  85
, 0, 255,  85
,  43, 0,  85
,  43,  43,  85
,  43,  85,  85
,  43, 128,  85
,  43, 170,  85
,  43, 213,  85
,  43, 255,  85
,  85, 0,  85
,  85,  43,  85
,  85, 128,  85
,  85, 170,  85
,  85, 213,  85
,  85, 255,  85
, 128, 0,  85
, 128,  43,  85
, 128,  85,  85
, 128, 128,  85
, 128, 170,  85
, 128, 213,  85
, 128, 255,  85
, 170, 0,  85
, 170,  43,  85
, 170,  85,  85
, 170, 128,  85
, 170, 170,  85
, 170, 213,  85
, 170, 255,  85
, 213, 0,  85
, 213,  43,  85
, 213,  85,  85
, 213, 128,  85
, 213, 170,  85
, 213, 213,  85
, 213, 255,  85
, 255, 0,  85
, 255,  43,  85
, 255,  85,  85
, 255, 128,  85
, 255, 170,  85
, 255, 213,  85
, 255, 255,  85
, 0,  43, 128
, 0,  85, 128
, 0, 128, 128
, 0, 170, 128
, 0, 213, 128
, 0, 255, 128
,  43, 0, 128
,  43,  43, 128
,  43,  85, 128
,  43, 128, 128
,  43, 170, 128
,  43, 213, 128
,  43, 255, 128
,  85, 0, 128
,  85,  43, 128
,  85,  85, 128
,  85, 128, 128
,  85, 170, 128
,  85, 213, 128
,  85, 255, 128
, 128, 0, 128
, 128,  43, 128
, 128,  85, 128
, 128, 170, 128
, 128, 213, 128
, 128, 255, 128
, 170, 0, 128
, 170,  43, 128
, 170,  85, 128
, 170, 128, 128
, 170, 170, 128
, 170, 213, 128
, 170, 255, 128
, 213, 0, 128
, 213,  43, 128
, 213,  85, 128
, 213, 128, 128
, 213, 170, 128
, 213, 213, 128
, 213, 255, 128
, 255, 0, 128
, 255,  43, 128
, 255,  85, 128
, 255, 128, 128
, 255, 170, 128
, 255, 213, 128
, 255, 255, 128
, 0,  43, 170
, 0,  85, 170
, 0, 128, 170
, 0, 170, 170
, 0, 213, 170
, 0, 255, 170
,  43, 0, 170
,  43,  43, 170
,  43,  85, 170
,  43, 128, 170
,  43, 170, 170
,  43, 213, 170
,  43, 255, 170
,  85, 0, 170
,  85,  43, 170
,  85,  85, 170
,  85, 128, 170
,  85, 170, 170
,  85, 213, 170
,  85, 255, 170
, 128, 0, 170
, 128,  43, 170
, 128,  85, 170
, 128, 128, 170
, 128, 170, 170
, 128, 213, 170
, 128, 255, 170
, 170, 0, 170
, 170,  43, 170
, 170,  85, 170
, 170, 128, 170
, 170, 213, 170
, 170, 255, 170
, 213, 0, 170
, 213,  43, 170
, 213,  85, 170
, 213, 128, 170
, 213, 170, 170
, 213, 213, 170
, 213, 255, 170
, 255, 0, 170
, 255,  43, 170
, 255,  85, 170
, 255, 128, 170
, 255, 170, 170
, 255, 213, 170
, 255, 255, 170
, 0,  43, 213
, 0,  85, 213
, 0, 128, 213
, 0, 170, 213
, 0, 213, 213
, 0, 255, 213
,  43, 0, 213
,  43,  43, 213
,  43,  85, 213
,  43, 128, 213
,  43, 170, 213
,  43, 213, 213
,  43, 255, 213
,  85, 0, 213
,  85,  43, 213
,  85,  85, 213
,  85, 128, 213
,  85, 170, 213
,  85, 213, 213
,  85, 255, 213
, 128, 0, 213
, 128,  43, 213
, 128,  85, 213
, 128, 128, 213
, 128, 170, 213
, 128, 213, 213
, 128, 255, 213
, 170, 0, 213
, 170,  43, 213
, 170,  85, 213
, 170, 128, 213
, 170, 170, 213
, 170, 213, 213
, 170, 255, 213
, 213, 0, 213
, 213,  43, 213
, 213,  85, 213
, 213, 128, 213
, 213, 170, 213
, 213, 255, 213
, 255, 0, 213
, 255,  43, 213
, 255,  85, 213
, 255, 128, 213
, 255, 170, 213
, 255, 213, 213
, 255, 255, 213
, 0,  43, 255
, 0,  85, 255
, 0, 128, 255
, 0, 170, 255
, 0, 213, 255
, 0, 255, 255
,  43, 0, 255
,  43,  43, 255
,  43,  85, 255
,  43, 128, 255
,  43, 170, 255
,  43, 213, 255
,  43, 255, 255
,  85, 0, 255
,  85,  43, 255
,  85,  85, 255
,  85, 128, 255
,  85, 170, 255
,  85, 213, 255
,  85, 255, 255
, 128, 0, 255
, 128,  43, 255
, 128,  85, 255
, 128, 128, 255
, 128, 170, 255
, 128, 213, 255
, 128, 255, 255
, 170, 0, 255
, 170,  43, 255
, 170,  85, 255
, 170, 128, 255
, 170, 170, 255
, 170, 213, 255
, 170, 255, 255
, 213, 0, 255
, 213,  43, 255
, 213,  85, 255
, 213, 128, 255
, 213, 170, 255
, 213, 213, 255
, 213, 255, 255
, 255, 0, 255
, 255,  43, 255
, 255,  85, 255
, 255, 128, 255
, 255, 170, 255
, 255, 213, 255
, 5, 0, 0
,  11, 0, 0
,  16, 0, 0
,  21, 0, 0
,  27, 0, 0
,  32, 0, 0
,  37, 0, 0
,  43, 0, 0
,  48, 0, 0
,  53, 0, 0
,  58, 0, 0
,  64, 0, 0
,  69, 0, 0
,  74, 0, 0
,  80, 0, 0
,  85, 0, 0
,  90, 0, 0
,  96, 0, 0
, 101, 0, 0
, 106, 0, 0
, 112, 0, 0
, 117, 0, 0
, 122, 0, 0
, 127, 0, 0
, 133, 0, 0
, 138, 0, 0
, 143, 0, 0
, 149, 0, 0
, 154, 0, 0
, 159, 0, 0
, 165, 0, 0
, 170, 0, 0
, 175, 0, 0
, 181, 0, 0
, 186, 0, 0
, 191, 0, 0
, 197, 0, 0
, 202, 0, 0
, 207, 0, 0
, 213, 0, 0
, 218, 0, 0
, 223, 0, 0
, 228, 0, 0
, 234, 0, 0
, 239, 0, 0
, 244, 0, 0
, 250, 0, 0
, 255, 0, 0
, 0, 5, 0
, 0,  11, 0
, 0,  16, 0
, 0,  21, 0
, 0,  27, 0
, 0,  32, 0
, 0,  37, 0
, 0,  43, 0
, 0,  48, 0
, 0,  53, 0
, 0,  58, 0
, 0,  64, 0
, 0,  69, 0
, 0,  74, 0
, 0,  80, 0
, 0,  85, 0
, 0,  90, 0
, 0,  96, 0
, 0, 101, 0
, 0, 106, 0
, 0, 112, 0
, 0, 117, 0
, 0, 122, 0
, 0, 127, 0
, 0, 133, 0
, 0, 138, 0
, 0, 143, 0
, 0, 149, 0
, 0, 154, 0
, 0, 159, 0
, 0, 165, 0
, 0, 170, 0
, 0, 175, 0
, 0, 181, 0
, 0, 186, 0
, 0, 191, 0
, 0, 197, 0
, 0, 202, 0
, 0, 207, 0
, 0, 213, 0
, 0, 218, 0
, 0, 223, 0
, 0, 228, 0
, 0, 234, 0
, 0, 239, 0
, 0, 244, 0
, 0, 250, 0
, 0, 255, 0
, 0, 0, 5
, 0, 0,  11
, 0, 0,  16
, 0, 0,  21
, 0, 0,  27
, 0, 0,  32
, 0, 0,  37
, 0, 0,  43
, 0, 0,  48
, 0, 0,  53
, 0, 0,  58
, 0, 0,  64
, 0, 0,  69
, 0, 0,  74
, 0, 0,  80
, 0, 0,  85
, 0, 0,  90
, 0, 0,  96
, 0, 0, 101
, 0, 0, 106
, 0, 0, 112
, 0, 0, 117
, 0, 0, 122
, 0, 0, 127
, 0, 0, 133
, 0, 0, 138
, 0, 0, 143
, 0, 0, 149
, 0, 0, 154
, 0, 0, 159
, 0, 0, 165
, 0, 0, 170
, 0, 0, 175
, 0, 0, 181
, 0, 0, 186
, 0, 0, 191
, 0, 0, 197
, 0, 0, 202
, 0, 0, 207
, 0, 0, 213
, 0, 0, 218
, 0, 0, 223
, 0, 0, 228
, 0, 0, 234
, 0, 0, 239
, 0, 0, 244
, 0, 0, 250
, 0, 0, 255
, 0, 0, 0
, 5, 5, 5
,  10,  10,  10
,  16,  16,  16
,  21,  21,  21
,  26,  26,  26
,  31,  31,  31
,  36,  36,  36
,  42,  42,  42
,  47,  47,  47
,  52,  52,  52
,  57,  57,  57
,  62,  62,  62
,  68,  68,  68
,  73,  73,  73
,  78,  78,  78
,  83,  83,  83
,  88,  88,  88
,  94,  94,  94
,  99,  99,  99
, 104, 104, 104
, 109, 109, 109
, 114, 114, 114
, 120, 120, 120
, 125, 125, 125
, 130, 130, 130
, 135, 135, 135
, 141, 141, 141
, 146, 146, 146
, 151, 151, 151
, 156, 156, 156
, 161, 161, 161
, 167, 167, 167
, 172, 172, 172
, 177, 177, 177
, 182, 182, 182
, 187, 187, 187
, 193, 193, 193
, 198, 198, 198
, 203, 203, 203
, 208, 208, 208
, 213, 213, 213
, 219, 219, 219
, 224, 224, 224
, 229, 229, 229
, 234, 234, 234
, 239, 239, 239
, 245, 245, 245
, 250, 250, 250
, 255, 255, 255
};





} // namespace rtabmap
