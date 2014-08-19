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

#include "utilite/UMp3Encoder.h"
#include <lame.h>
#include <utilite/UFile.h>
#include "utilite/UWav.h"

int UMp3Encoder::encode(const std::string &fileIn, const std::string &fileOut, bool writeOver)
{
    if(!UFile::exists(fileIn))
    {
        return -1;
    }

    if(!writeOver && UFile::exists(fileOut))
    {
        return -1;
    }

    int ret_code;
    int num_samples;
    unsigned char *mp3buffer = 0;
    int  mp3buffer_size;
    UWav wav;
    FILE* fpOut = 0;

    fpOut = fopen(fileOut.c_str(), "w+b");
    if(fpOut == NULL)
    {
        return 1;
    }

    // Load the wav
    if(wav.init(fileIn.c_str()) != 0)
    {
        return 2;
    }

    // Support only 1 or 2 channels
    if(wav.getChannels() != 1 && wav.getChannels() != 2)
    {
        return 3;
    }


    // See "API" file in lame source directory or "lame.h"
    //

    // 1. (optional) Get the version number of the encoder, if you are interested.  
    //void get_lame_version(char *strbuf, size_t buflen, const char *prefix);


    // 2. Error messages.  By default, LAME will write error messages to
    // stderr using vfprintf().  For GUI applications, this is often a problem
    // and you need to set your own error message handlers:

    //lame_set_errorf(gfp,error_handler_function);
    //lame_set_debugf(gfp,error_handler_function);
    //lame_set_msgf(gfp,error_handler_function);

    //3. Initialize the encoder.  sets default for all encoder parameters.

    lame_global_flags *gfp;
    gfp = lame_init();

    // The default (if you set nothing) is a  J-Stereo, 44.1khz
    // 128kbps CBR mp3 file at quality 5.  Override various default settings 
    // as necessary, for example:

    lame_set_num_samples(gfp, wav.getNumSamplesPerChannel());
    lame_set_num_channels(gfp, wav.getChannels()); // 1
    lame_set_in_samplerate(gfp, wav.getSamplesPerSec()); // 44100
    //lame_set_brate(gfp,128);
    //lame_set_mode(gfp, MONO); // 3 = mono
    //lame_set_quality(gfp,2);   /* 2=high  5 = medium  7=low */


    //4. Set more internal configuration based on data provided above,
    // as well as checking for problems.  Check that ret_code >= 0.

    ret_code = lame_init_params(gfp);
    if(ret_code<0)
    {
        return 4;
    }

    // 5. Encode some data.  input pcm data, output (maybe) mp3 frames.
    // This routine handles all buffering, resampling and filtering for you.
    // The required mp3buffer_size can be computed from num_samples, 
    // samplerate and encoding rate, but here is a worst case estimate:
    // mp3buffer_size (in bytes) = 1.25*num_samples + 7200.
    // num_samples = the number of PCM samples in each channel.  It is
    // not the sum of the number of samples in the L and R channels.

    // The return code = number of bytes output in mp3buffer.  This can be 0.
    // If it is <0, an error occured.

    encodeProcessBegin(wav.getNumTotalSamples());

    num_samples = wav.getNumSamplesPerChannel();

    mp3buffer_size = (num_samples*4)/3 + 7200;
    mp3buffer = new unsigned char[mp3buffer_size];

    // interleaved -> 1152*2 total samples (for 2 channels)
    long samplesSize = 1152*2;
    short* samples = new short[samplesSize];
    int iread = 0;
    long samplesWrited = 0;
    do {
         
        iread = wav.readNextSamples(samples, samplesSize);
        if (iread >= 0) {
            /* encode */
            // iread divide by 2 because we want samples by channels, not all samples read
            ret_code = lame_encode_buffer_interleaved(gfp, samples, iread/2,
                                          mp3buffer, mp3buffer_size);
            // Write data
            fwrite(mp3buffer, 1, ret_code, fpOut);
        }
        samplesWrited += iread;
        encodeProcess(samplesWrited);
    } while (iread > 0);

    // There are also routines for various types of input  
    // (float, long, interleaved, etc).  See lame.h for details.


    // 6. lame_encode_flush will flush the buffers and may return a 
    // final few mp3 frames.  mp3buffer should be at least 7200 bytes.
    // return code = number of bytes output to mp3buffer.  This can be 0.

    ret_code = lame_encode_flush(gfp, mp3buffer, mp3buffer_size);
    if(ret_code < 0)
    {
        delete samples;
        delete mp3buffer;
        encodeProcessEnd();
        return 6;
    }

    // Write last frame to file
    fwrite(mp3buffer, 1, ret_code, fpOut);

    // 7.  Write the Xing VBR/INFO tag to mp3 file.  
    lame_mp3_tags_fid(gfp, fpOut);

    // This adds a valid mp3 frame which contains information about the
    // bitstream some players may find usefull.  It is used for CBR,ABR and
    // VBR.  The routine will attempt to rewind the output stream to the
    // beginning.  If this is not possible, (for example, you are encoding to
    // stdout) you should specifically disable the tag by calling
    // lame_set_bWriteVbrTag(gfp,0) in step 3 above, and call
    // lame_mp3_tags_fid() with fid=NULL.  If the rewind fails and
    // the tag was not disabled, the first mp3 frame in the bitstream
    // will be all 0's.


    // 8. free the internal data structures.
    lame_close(gfp); 


    //
    fclose(fpOut);

    //
    delete samples;
    delete mp3buffer;

    encodeProcessEnd();

    return 0;
}
