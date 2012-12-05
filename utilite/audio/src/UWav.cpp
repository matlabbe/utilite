#include "utilite/UWav.h"
#include <utilite/ULogger.h>

#include <stdlib.h>
#include <string.h>

// Define (pour la fonction WriteWavHeader)
#if defined(WIN32) || defined(__WATCOMC__) || defined(_WIN32) || defined(__WIN32__)
    #define __PACKED
#else
    #define __PACKED __attribute__((packed))
#endif

void UWav::writeWavHeader(FILE *fp, unsigned int length, float rate, int channels, int bits)
{
	UDEBUG("");

	if(fp == 0)
	{
		UERROR("File pointer is null!");
		return;
	}

	fseek(fp, 0, SEEK_SET);

	{
		#if defined(WIN32) || defined(_WIN64) || defined(__WATCOMC__) || defined(_WIN32) || defined(__WIN32__)
		#pragma pack(1)
		#endif

		//    WAV Structures
		typedef struct
		{
			signed char id[4];
			int         size;
		} RiffChunk;

		struct
		{
			RiffChunk       chunk;
			unsigned short    wFormatTag      __PACKED;    /* format type  */
			unsigned short    nChannels       __PACKED;    /* number of channels (i.e. mono, stereo...)  */
			unsigned int    nSamplesPerSec  __PACKED;    /* sample rate  */
			unsigned int    nAvgBytesPerSec __PACKED;    /* for buffer estimation  */
			unsigned short    nBlockAlign     __PACKED;    /* block size of data  */
			unsigned short    wBitsPerSample  __PACKED;    /* number of bits per sample of mono data */
		} FmtChunk  = { {{'f','m','t',' '}, sizeof(FmtChunk) - sizeof(RiffChunk) }, 1, channels, (int)rate, (int)rate * channels * bits / 8, 1 * channels * bits / 8, bits };

		struct
		{
			RiffChunk   chunk;
		} DataChunk = { {{'d','a','t','a'}, length } };

		struct
		{
			RiffChunk   chunk;
			signed char rifftype[4];
		} WavHeader = { {{'R','I','F','F'}, sizeof(FmtChunk) + sizeof(RiffChunk) + length }, {'W','A','V','E'} };

		#if defined(WIN32) || defined(_WIN64) || defined(__WATCOMC__) || defined(_WIN32) || defined(__WIN32__)
		#pragma pack()
		#endif

		//    Write out the WAV header.
		fwrite(&WavHeader, sizeof(WavHeader), 1, fp);
		fwrite(&FmtChunk, sizeof(FmtChunk), 1, fp);
		fwrite(&DataChunk, sizeof(DataChunk), 1, fp);
	}
}


UWav::UWav() : _fileName(""), _pFile(0), _initialized(false), _dataOffset(0)
{
    UDEBUG("");
    memset(&_wavHeader, 0, sizeof(_wavHeader));
    memset(&_fmtChunk, 0, sizeof(_fmtChunk));
    memset(&_dataChunk, 0, sizeof(_dataChunk));
}

UWav::~UWav()
{
    UDEBUG("");
    // terminate reading the file
    if(_pFile)
    	fclose (_pFile);
}

int UWav::init(const char* fileName)
{
    _fileName = fileName;

    // Declare some variables
    long fileSize = 0;
    long readOffset = 0;
    char buffer[5] = {0};
    int wavHeaderSize = sizeof(WavHeader) + sizeof(FmtChunk) + sizeof(ChunkHeader);

    // Read the file
    _pFile = fopen(_fileName , "rb" );
    if(_pFile==NULL)
    {
        return 1; // Can't oopen the file
    }

    // obtain file size:
    fseek (_pFile , 0 , SEEK_END);
    fileSize = ftell (_pFile);
    rewind (_pFile);

    if(fileSize < wavHeaderSize)
    {
        return 2; // File not valid, too small
    }

    readOffset += (long)fread (&_wavHeader, 1, sizeof(_wavHeader), _pFile);
    readOffset += (long)fread (&_fmtChunk, 1, sizeof(_fmtChunk), _pFile);

    // Is it a valid wav ?
    buffer[0] = _wavHeader.header.id[0];
    buffer[1] = _wavHeader.header.id[1];
    buffer[2] = _wavHeader.header.id[2];
    buffer[3] = _wavHeader.header.id[3];
    if(strcmp(buffer, "RIFF") != 0)
    {
        return 3;
    }
    buffer[0] = _wavHeader.rifftype[0];
    buffer[1] = _wavHeader.rifftype[1];
    buffer[2] = _wavHeader.rifftype[2];
    buffer[3] = _wavHeader.rifftype[3];

    if(strcmp(buffer, "WAVE") != 0)
    {
        return 4;
    }

    buffer[0] = _fmtChunk.header.id[0];
    buffer[1] = _fmtChunk.header.id[1];
    buffer[2] = _fmtChunk.header.id[2];
    buffer[3] = _fmtChunk.header.id[3];
    if(strcmp(buffer, "fmt ") != 0)
    {
        return 5;
    }
    
    // Read data chunk header
    readOffset += (long)fread (&_dataChunk, 1, sizeof(ChunkHeader), _pFile);

    buffer[0] = _dataChunk.header.id[0];
    buffer[1] = _dataChunk.header.id[1];
    buffer[2] = _dataChunk.header.id[2];
    buffer[3] = _dataChunk.header.id[3];
    if(strcmp(buffer, "data") != 0 && _dataChunk.header.size > 0)
    {
        return 6;
    }

    //
    if((fileSize - wavHeaderSize) < _dataChunk.header.size)
    {
        return 7; //
    }

    _dataOffset = readOffset;

    _initialized = true;

    return 0;

}

long UWav::readData(unsigned char data[], long offset, long dataLength)
{
    if(!_initialized)
    {
        return -1;
    }
    
    //
    if(fseek(_pFile, _dataOffset+offset, SEEK_SET) != 0)
    {
        return -2;
    }

    long pos = _dataOffset+offset;

    // pos is after the data chunk, so return 0
    if(pos > (_dataOffset + _dataChunk.header.size))
    {
        return 0; //0 byte read
    }

    // data left to be read
    if(dataLength > _dataChunk.header.size - pos)
    {
        dataLength = _dataChunk.header.size - pos;
    }

    return (long)fread (data, 1, dataLength, _pFile);
}

long UWav::readNextData(unsigned char data[], long dataLength)
{
    if(!_initialized)
    {
        return -1;
    }

    // pos is before the data chunk
    long pos = ftell(_pFile);
    if(pos < _dataOffset)
    {
        pos = _dataOffset;
        // go to the beginning of the data chunk
        if(fseek(_pFile, _dataOffset, SEEK_SET) != 0)
        {
            return -2;
        }
    }

    // pos is after the data chunk, so return 0
    if(pos > (_dataOffset + _dataChunk.header.size))
    {
        return 0; //0 byte read
    }

    // data left to be read
    if(dataLength > _dataChunk.header.size - pos)
    {
        dataLength = _dataChunk.header.size - pos;
    }

    return (long)fread (data, 1, dataLength, _pFile);
}

// read samples and return a table with 2 channels interleaved
// if the wav have 1 channel, a channel2 with all values to 0 is added.
long UWav::readNextSamples(short samples[], long numSamples)
{
    // Only supported 1 and 2 channels
    if(!_initialized || getChannels() < 1 || getChannels() > 2)
    {
        return -1;
    }

    long dataSize;
    if(getChannels() == 1)
    {
        dataSize = numSamples;
    }
    else // if(getChannels() == 1)
    {
        dataSize = numSamples * getBytesPerSample();
    }
    unsigned char* data = new unsigned char[dataSize];
    long bytesRead = readNextData(data, dataSize);
    if(bytesRead <= 0)
    {
        delete data;
        return bytesRead;
    }

    long samplesRead = bytesRead/getBytesPerSample();

    short tmp;

    if(getChannels() == 1)
    {
        samplesRead*=2; // we set values for channels 2 to 0;
        // Convert to 2 channels [channels1Data ; 0]
        for(int i=0; i<samplesRead;)
        {
            tmp = (short)data[i+1];
            tmp <<= 8;
            tmp |= (short)data[i];
            samples[i] = tmp;
            samples[i+1] = 0;
            i+=2;
        }
    }
    else // if(getChannels() == 2)
    {
        for(int i=0; i<samplesRead;)
        {
            tmp = (short)data[2*i+1];
            tmp <<= 8;
            tmp |= (short)data[2*i];
            samples[i] = tmp;

            tmp = (short)data[2*i+3];
            tmp <<= 8;
            tmp |= (short)data[2*i+2];
            samples[i+1] = tmp;
            i+=2;
        }
    }

    delete data;

    return samplesRead;
}

unsigned long UWav::getNumSamplesPerChannel() const
{
    // Avoid divide by 0
    if(_fmtChunk.wChannels != 0 && _fmtChunk.wBlockAlign != 0)
    {
        return (_dataChunk.header.size / _fmtChunk.wChannels) / getBytesPerSample();
    }

    return 0;
}
unsigned long UWav::getNumTotalSamples() const
{
    // Avoid divide by 0
    if(_fmtChunk.wBlockAlign != 0)
    {
        return _dataChunk.header.size / getBytesPerSample();
    }

    return 0;
}

unsigned short UWav::getBytesPerSample() const
{
    return (_fmtChunk.wBitsPerSample+7)/8;
}
