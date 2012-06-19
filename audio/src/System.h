/**
 * @file System.h
 *
 * @author Mathieu Labbe
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <utilite/UMutex.h>

#include <fmod.hpp>
#include <fmod_errors.h>

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
class System
{
public:  
    /**
     * Method used to release the FMOD system. Use it 
     * only when no more audio processes are running.
     */
    static void release();

    static void ERRCHECK(const FMOD_RESULT & result);

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
    static FMOD_RESULT isRecording(int driver, bool * isRecording);
    static FMOD_RESULT recordStart(int driver,
    							   FMOD::Sound * sound,
    							   bool loop);
    static FMOD_RESULT recordStop(int driver);
    static FMOD_RESULT getRecordPosition(int driver, unsigned int* position);

    static FMOD_RESULT createSound(const char * name_or_data,
						   FMOD_MODE mode,
						   FMOD_CREATESOUNDEXINFO * exinfo,
						   FMOD::Sound ** sound);
    static FMOD_RESULT createStream(const char * name_or_data,
							FMOD_MODE mode,
							FMOD_CREATESOUNDEXINFO * exinfo,
							FMOD::Sound ** sound);
    static FMOD_RESULT createDSPByType(FMOD_DSP_TYPE dspType,
                               FMOD::DSP ** dsp);

    static FMOD_RESULT playSound(FMOD_CHANNELINDEX channelIndex,
						 FMOD::Sound * sound,
						 bool paused,
						 FMOD::Channel ** channel);
    static FMOD_RESULT playDSP(FMOD_CHANNELINDEX channelIndex,
					   FMOD::DSP * dsp,
					   bool paused,
					   FMOD::Channel ** channel);

    static FMOD_RESULT update();
    // END - FMOD System encapsulated methods

private:
    static System* instance();
    System();
    ~System();
    void init();

private:
    UMutex _fmodSystemMutex;
    FMOD::System * _fmodSystem;
};

#endif
