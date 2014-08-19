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

#ifndef UWAV_H
#define UWAV_H

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include <stdio.h>

/**
 * This class represents a WAV file. When initiated, it 
 * reads the file headers and keep informations in his
 * attributes accessibles with the getters. The data aren't 
 * read at the initialization.
 */
class UTILITEAUDIO_EXP UWav
{

public:
	static void writeWavHeader(FILE *fp, unsigned int length, float rate, int channels, int bits);

public:
    UWav();
    ~UWav();

    /**
     * It reads the file headers to fill wav structures.
     * @param fileName the file name
     * @return int 0 if success, otherwise an error code
     */
    int init(const char* fileName);

    //getters
    unsigned short getChannels() const       {return _fmtChunk.wChannels;}
    unsigned long getSamplesPerSec() const   {return _fmtChunk.dwSamplesPerSec;}
    unsigned long getAvgBytesPerSec() const  {return _fmtChunk.dwAvgBytesPerSec;}
    unsigned short getBlockAlign() const     {return _fmtChunk.wBlockAlign;}
    unsigned short getBitsPerSample() const  {return _fmtChunk.wBitsPerSample;}

    unsigned long getDataLength() const      {return _dataChunk.header.size;}

    unsigned long getNumSamplesPerChannel() const;
    unsigned long getNumTotalSamples() const;
    unsigned short getBytesPerSample() const;

    /**
     * Read the data from the offset.
     * @param data buffer for the data read
     * @param offset offset where the data begin to be read.
     * @param dataLength length of the data buffer
     * @return long bytes read
     * 
     */
    long readData(unsigned char data[], 
                  long offset, 
                  long dataLength);

    /**
     * Read the next data depending on the 
     * last call. First call to this method will 
     * read from the beginning of the data chunk.
     * @param data buffer for the data read
     * @param dataLength length of the data buffer
     * @return long bytes read
     */
    long readNextData(unsigned char data[], 
                      long dataLength);

    /**
     * Read the next samples (16 bits per samples).
     * @param samples buffer for the samples read
     * @param numSamples length of the samples buffer
     * @return long samples read
     */
    long readNextSamples(short samples[], 
                         long numSamples);

protected:

private:
    // Define wav structures
    struct ChunkHeader {
        signed char id[4];
        int        size;
    } ;

    struct WavHeader {
        ChunkHeader      header;
        signed char  rifftype[4];
    };

    struct FmtChunk {
      ChunkHeader        header;
      short          wFormatTag;
      unsigned short wChannels;
      unsigned int  dwSamplesPerSec;
      unsigned int  dwAvgBytesPerSec;
      unsigned short wBlockAlign;
      unsigned short wBitsPerSample;
      /* Note: there may be additional fields here, depending upon wFormatTag. */
    };

    struct DataChunk{
      ChunkHeader        header;
    };

private:
    WavHeader _wavHeader;
    FmtChunk _fmtChunk;
    DataChunk _dataChunk;

    const char* _fileName;
    FILE* _pFile;
    bool _initialized;
    long _dataOffset;
};

#endif
