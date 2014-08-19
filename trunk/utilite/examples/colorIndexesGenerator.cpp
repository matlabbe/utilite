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

#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp>
#include <iostream>
#include <fstream>
#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include "utilite/UColorTable.h"
#include <zlib.h>
#include <utilite/UConversion.h>

#define FILE_NAME_PREFIX "ColorIndexes"
#define FILE_NAME_SUFFIX ".bin"
#define CHUNK 16384

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-colorIndexesGenerator size\n"
			"  size          Supported : 8, 16, 32, 64, 128, 256, 512,\n"
			"                            1024, 2048, 4096, 8192, 16384,\n"
			"                            32768, 65536\n");
	exit(1);
}

cv::Mat generateColorTable()
{
	cv::Mat colorTable(256*256*256, 3, CV_32F);
	for(int b=0; b<256; ++b)
	{
		for(int g=0; g<256; ++g)
		{
			for(int r=0; r<256; ++r)
			{
				colorTable.at<float>(b*256*256 + g*256 + r, 0) = (float)r;
				colorTable.at<float>(b*256*256 + g*256 + r, 1) = (float)g;
				colorTable.at<float>(b*256*256 + g*256 + r, 2) = (float)b;
				UDEBUG("r=%f, g=%f, b=%f", (float)r , (float)g, (float)b);
			}
		}
	}
	return colorTable;
}

/* report a zlib or i/o error */
void zerr(int ret)
{
    fputs("zpipe: ", stderr);
    switch (ret) {
    case Z_ERRNO:
        if (ferror(stdin))
            fputs("error reading stdin\n", stderr);
        if (ferror(stdout))
            fputs("error writing stdout\n", stderr);
        break;
    case Z_STREAM_ERROR:
        fputs("invalid compression level\n", stderr);
        break;
    case Z_DATA_ERROR:
        fputs("invalid or incomplete deflate data\n", stderr);
        break;
    case Z_MEM_ERROR:
        fputs("out of memory\n", stderr);
        break;
    case Z_VERSION_ERROR:
        fputs("zlib version mismatch!\n", stderr);
        break;
    }
    UFATAL("");
}

/* Compress from file source to file dest until EOF on source.
   def() returns Z_OK on success, Z_MEM_ERROR if memory could not be
   allocated for processing, Z_STREAM_ERROR if an invalid compression
   level is supplied, Z_VERSION_ERROR if the version of zlib.h and the
   version of the library linked do not match, or Z_ERRNO if there is
   an error reading or writing the files. */
int def(FILE *source, FILE *dest, int level)
{
    int ret, flush;
    unsigned have;
    z_stream strm;
    unsigned char in[CHUNK];
    unsigned char out[CHUNK];

    /* allocate deflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    ret = deflateInit(&strm, level);
    if (ret != Z_OK)
        return ret;
    UDEBUG("");
    /* compress until end of file */
    do {
    	UDEBUG("");
        strm.avail_in = fread(in, 1, CHUNK, source);
        if (ferror(source)) {
            (void)deflateEnd(&strm);
            return Z_ERRNO;
        }
        flush = feof(source) ? Z_FINISH : Z_NO_FLUSH;
        strm.next_in = in;

        /* run deflate() on input until output buffer not full, finish
           compression if all of source has been read in */
        do {
        	UDEBUG("");
            strm.avail_out = CHUNK;
            strm.next_out = out;
            ret = deflate(&strm, flush);    /* no bad return value */
            assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
            have = CHUNK - strm.avail_out;
            if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
                (void)deflateEnd(&strm);
                return Z_ERRNO;
            }
        } while (strm.avail_out == 0);
        assert(strm.avail_in == 0);     /* all input will be used */

        /* done when last data in file processed */
    } while (flush != Z_FINISH);
    assert(ret == Z_STREAM_END);        /* stream will be complete */

    /* clean up and return */
    (void)deflateEnd(&strm);
    return Z_OK;
}

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);
	ULogger::setPrintWhere(false);

	UTimer timer;

	if(argc<2)
	{
		showUsage();
	}
	int size = atoi(argv[1]);

	cv::Mat colorTable;
	switch(size)
	{
	case UColorTable::kSize8:
		colorTable = cv::Mat(UColorTable::kSize8, 3, CV_8U, UColorTable::INDEXED_TABLE_8);
		break;
	case UColorTable::kSize16:
		colorTable = cv::Mat(UColorTable::kSize16, 3, CV_8U, UColorTable::INDEXED_TABLE_16);
		break;
	case UColorTable::kSize32:
		colorTable = cv::Mat(UColorTable::kSize32, 3, CV_8U, UColorTable::INDEXED_TABLE_32);
		break;
	case UColorTable::kSize64:
		colorTable = cv::Mat(UColorTable::kSize64, 3, CV_8U, UColorTable::INDEXED_TABLE_64);
		break;
	case UColorTable::kSize128:
		colorTable = cv::Mat(UColorTable::kSize128, 3, CV_8U, UColorTable::INDEXED_TABLE_128);
		break;
	case UColorTable::kSize256:
		colorTable = cv::Mat(UColorTable::kSize256, 3, CV_8U, UColorTable::INDEXED_TABLE_256);
		break;
	case UColorTable::kSize512:
		colorTable = cv::Mat(UColorTable::kSize512, 3, CV_8U, UColorTable::INDEXED_TABLE_512);
		break;
	case UColorTable::kSize1024:
		colorTable = cv::Mat(UColorTable::kSize1024, 3, CV_8U, UColorTable::INDEXED_TABLE_1024);
		break;
	case UColorTable::kSize2048:
		colorTable = cv::Mat(UColorTable::kSize2048, 3, CV_8U, UColorTable::INDEXED_TABLE_2048);
		break;
	case UColorTable::kSize4096:
		colorTable = cv::Mat(UColorTable::kSize4096, 3, CV_8U, UColorTable::INDEXED_TABLE_4096);
		break;
	case UColorTable::kSize8192:
		colorTable = cv::Mat(UColorTable::kSize8192, 3, CV_8U, UColorTable::INDEXED_TABLE_8192);
		break;
	case UColorTable::kSize16384:
		colorTable = cv::Mat(UColorTable::kSize16384, 3, CV_8U, UColorTable::INDEXED_TABLE_16384);
		break;
	case UColorTable::kSize32768:
		colorTable = cv::Mat(UColorTable::kSize32768, 3, CV_8U, UColorTable::INDEXED_TABLE_32768);
		break;
	case UColorTable::kSize65536:
		colorTable = cv::Mat(UColorTable::kSize65536, 3, CV_8U, UColorTable::INDEXED_TABLE_65536);
		break;
	default:
		printf("\nSize not supported...\n");
		showUsage();
		break;
	}

	cv::Mat data;
	colorTable.convertTo(data, CV_32F);

	if(data.rows > 0x10000)
	{
		UFATAL("Color table is too big (%d > 2^16)", data.rows);
	}

	UINFO("Generating full color cube...");
	cv::Mat queries = generateColorTable();
	UINFO("Generating full color cube... done!");

	cv::Mat dists(queries.rows, 1, CV_32F);
	cv::Mat indices = cv::Mat(queries.rows, 1, CV_32S);

	UINFO("Nearest neighbor searching (queries=%d, indexes=%d)...", queries.rows, data.rows);
	cv::flann::Index flannIndex(data, cv::flann::LinearIndexParams()); // Don't use approximate nn
	flannIndex.knnSearch(queries, indices, dists, 1);
	UINFO("Nearest neighbor searching (queries=%d, indexes=%d)... done!", queries.rows, data.rows);

	std::string fileName = uFormat("%s%d%s", FILE_NAME_PREFIX, size, FILE_NAME_SUFFIX);
	UINFO("Saving color indexed table to %s...", fileName.c_str());
	std::ofstream outfile(fileName.c_str(), std::ios_base::out | std::ios_base::binary);
	unsigned short index;
	for(int i=0; i<indices.rows; ++i)
	{
		UDEBUG("%d = %d", i, indices.at<int>(i,0));
		index = (unsigned short)indices.at<int>(i,0); // assume indexes are under 2^16
		outfile.write((const char *)&index, sizeof(unsigned short));
	}
	outfile.close();
	UINFO("Saving color indexed table to %s... done!", fileName.c_str());

	std::string zipFileName = fileName + std::string(".zip");
	UINFO("Compressing file %s to %s...", fileName.c_str(), zipFileName.c_str());
	FILE * input = fopen(fileName.c_str(), "rb");
	FILE * compressed = fopen(zipFileName.c_str(), "wb");

	int result = def(input, compressed, Z_DEFAULT_COMPRESSION);
	if(result != Z_OK)
	{
		zerr(result);
		UERROR("");
	}

	fclose(input);
	fclose(compressed);
	UINFO("Compressing file %s to %s... done!", fileName.c_str(), zipFileName.c_str());

	UINFO("Total time = %f s", timer.getElapsedTime());
    return 0;
}
