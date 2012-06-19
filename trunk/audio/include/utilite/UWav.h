/**
 * @file UWav.h
 *
 * @author Mathieu Labbe
 */

#ifndef UWAV_H
#define UWAV_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <stdio.h>

/**
 * This class represents a WAV file. When initiated, it 
 * reads the file headers and keep informations in his
 * attributes accessibles with the getters. The data aren't 
 * read at the initialization.
 */
class UTILITE_EXP UWav
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
