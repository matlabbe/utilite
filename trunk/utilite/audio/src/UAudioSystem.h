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

#ifndef SYSTEM_H
#define SYSTEM_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <utilite/UMutex.h>

#include <fmodex/fmod.hpp>
#include <fmodex/fmod_errors.h>

/**
 * A sound system is used to encapsulate FMOD methods 
 * and to protect access to FMOD system for 
 * multi-threaded applications.
 * It is a Singleton but only friend classes 
 * have access to FMOD methods. An application should 
 * use release() method before end to release 
 * normally the FMOD system. 
 *
 * @see System::release
 *
 */
class UAudioSystem
{
public:  

	static void acquire();
    /**
     * Method used to release the FMOD system. Use it
     * only when no more audio processes are running.
     */
    static void release();

    ///////////////////////////////////////
    // FMOD System encapsulated methods
    //
    // These methods are used to protect access 
    // to system methods with a mutex.
    ///////////////////////////////////////
    static FMOD_RESULT getRecordNumDrivers(int* numDrivers);
    static FMOD_RESULT getRecordDriverInfo(int id,
                                           char* name, 
                                           int namelen, 
                                           FMOD_GUID* guid);
    static FMOD_RESULT isRecording(int driver, FMOD_BOOL * isRecording);
    static FMOD_RESULT recordStart(int driver,
    							   FMOD_SOUND * sound,
    							   bool loop);
    static FMOD_RESULT recordStop(int driver);
    static FMOD_RESULT getRecordPosition(int driver, unsigned int* position);

    static FMOD_RESULT createSound(const char * name_or_data,
						   FMOD_MODE mode,
						   FMOD_CREATESOUNDEXINFO * exinfo,
						   FMOD_SOUND ** sound);
    static FMOD_RESULT createStream(const char * name_or_data,
							FMOD_MODE mode,
							FMOD_CREATESOUNDEXINFO * exinfo,
							FMOD_SOUND ** sound);
    static FMOD_RESULT createDSPByType(FMOD_DSP_TYPE dspType,
                               FMOD_DSP ** dsp);

    static FMOD_RESULT playSound(FMOD_CHANNELINDEX channelIndex,
    					 FMOD_SOUND * sound,
						 bool paused,
						 FMOD_CHANNEL ** channel);
    static FMOD_RESULT playDSP(FMOD_CHANNELINDEX channelIndex,
					   FMOD_DSP * dsp,
					   bool paused,
					   FMOD_CHANNEL ** channel);

    static FMOD_RESULT update();
    // END - FMOD System encapsulated methods

private:
    static UAudioSystem* instance();
    UAudioSystem();
    ~UAudioSystem();
    void init();

private:
    UMutex _fmodSystemMutex;
    FMOD_SYSTEM * _fmodSystem;
    static int _objectsUsingAudioSystemCount;
};

#endif
