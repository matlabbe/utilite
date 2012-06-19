#include "UAudioSystem.h"
#include <utilite/ULogger.h>

int UAudioSystem::_objectsUsingAudioSystemCount = 0;

//=========================
//public:
//=========================
void UAudioSystem::acquire()
{
	UDEBUG("_objectsUsingAudioSystemCount=%d", _objectsUsingAudioSystemCount);
	++_objectsUsingAudioSystemCount;
	if(_objectsUsingAudioSystemCount == 1)
	{
		instance();// this will init the fmod system
	}
}

void UAudioSystem::release()
{
	UDEBUG("_objectsUsingAudioSystemCount=%d", _objectsUsingAudioSystemCount);
	--_objectsUsingAudioSystemCount;
	if(_objectsUsingAudioSystemCount == 0)
	{
		// the last release closes the fmod system (avoiding freezing problem below in destructor)
		FMOD_RESULT result;
		UAudioSystem* system = UAudioSystem::instance();
		if(system->_fmodSystem) {
			system->_fmodSystemMutex.lock();
			result = FMOD_System_Close(system->_fmodSystem);   UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
			result = FMOD_System_Release(system->_fmodSystem); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
			system->_fmodSystem = 0;
			system->_fmodSystemMutex.unlock();
		}
	}
}

//=========================
//private:
//=========================
UAudioSystem* UAudioSystem::instance()
{  
    static UAudioSystem system;
    if(system._fmodSystem == 0)
    {
        system._fmodSystemMutex.lock();
        system.init();
        system._fmodSystemMutex.unlock();
    }
    return &system;
}

UAudioSystem::UAudioSystem() : _fmodSystem(0)
{
    UDEBUG("");
    init();
}

UAudioSystem::~UAudioSystem()
{
    UDEBUG("");
    // Probleme ici lorsque l'application termine, _fmodSystem->close() plante...
    // Appeler à la place System::release() dans le main
    FMOD_RESULT result;
    if(_fmodSystem)
    {
    	UERROR("The audio system should already be released here (using UAudioSystem::release())... may freeze on exit!");
    	_fmodSystemMutex.lock();
    	result = FMOD_System_Close(_fmodSystem);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    	result = FMOD_System_Release(_fmodSystem);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
        _fmodSystem = 0;
        _fmodSystemMutex.unlock();
    }
}

void UAudioSystem::init()
{
    if(_fmodSystem) {
        return;
    }
    UDEBUG("");

    FMOD_RESULT result;

    //  Create a System object.
    result = FMOD_System_Create(&_fmodSystem);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    //TODO : faire un check de version de FMOD
    unsigned int version;
    result = FMOD_System_GetVersion(_fmodSystem, &version);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    if (version < FMOD_VERSION)    //Vérification de la version de la librairie FMOD du systmème
    {
        UFATAL("Erreur!  Vous utilisez une vieille version de FMOD %08x.  Ce programme requiere la version %08x", version, FMOD_VERSION);
    }


    //Set output device
    UDEBUG("Auto detecting the output device...");
#ifdef WIN32
    FMOD_System_SetOutput(_fmodSystem, FMOD_OUTPUTTYPE_DSOUND); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
#else
    FMOD_System_SetOutput(_fmodSystem, FMOD_OUTPUTTYPE_AUTODETECT); UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
#endif
    //
    //_fmodSystem->setOutput(FMOD_OUTPUTTYPE_WASAPI);            UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));

    FMOD_OUTPUTTYPE output;
    FMOD_System_GetOutput(_fmodSystem, &output);

    //Log output device
    if(output == FMOD_OUTPUTTYPE_DSOUND)
    {
    	UDEBUG("Using output device = FMOD_OUTPUTTYPE_DSOUND");
    }
    else if(output == FMOD_OUTPUTTYPE_WASAPI)
    {
        UDEBUG("Using output device = FMOD_OUTPUTTYPE_WASAPI");
    }
    else
    {
    	UDEBUG("Using output device = %d", output);
    }


    result = FMOD_System_Init(_fmodSystem, 32, FMOD_INIT_NORMAL, 0);        UASSERT_MSG(result==FMOD_OK, FMOD_ErrorString(result));
    UDEBUG("");
}

//=========================
//protected:
//=========================
///////////////////////////////////////
// FMOD System encapsulated methods
//
// These methods are used to protect access 
// to system methos with a mutex.
///////////////////////////////////////
FMOD_RESULT UAudioSystem::getRecordNumDrivers(int * numDrivers)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_GetRecordNumDrivers(system->_fmodSystem, numDrivers);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::getRecordDriverInfo(int id,
                                char * name,
                                int namelen, 
                                FMOD_GUID* guid)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_GetRecordDriverInfo(system->_fmodSystem, id, name, namelen, guid);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::isRecording(int driver, FMOD_BOOL * isRecording)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_IsRecording(system->_fmodSystem, driver, isRecording);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::recordStart(int driver,
						FMOD_SOUND * sound,
						bool loop)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_RecordStart(system->_fmodSystem, driver, sound, loop);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::recordStop(int driver)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_RecordStop(system->_fmodSystem, driver);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::getRecordPosition(int driver, unsigned int * position)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_GetRecordPosition(system->_fmodSystem, driver, position);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::createSound(const char * name_or_data,
                        FMOD_MODE mode,
                        FMOD_CREATESOUNDEXINFO * exinfo,
                        FMOD_SOUND ** sound)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_CreateSound(system->_fmodSystem, name_or_data, mode, exinfo, sound);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::createStream(const char * name_or_data,
                                   FMOD_MODE mode,
                                   FMOD_CREATESOUNDEXINFO * exinfo,
                                   FMOD_SOUND ** sound)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_CreateSound(system->_fmodSystem, name_or_data, mode, exinfo, sound);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}
FMOD_RESULT UAudioSystem::createDSPByType(FMOD_DSP_TYPE dspType,
                            FMOD_DSP ** dsp)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_CreateDSPByType(system->_fmodSystem, dspType, dsp);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::playSound(FMOD_CHANNELINDEX channelIndex,
						FMOD_SOUND * sound,
						bool paused,
						FMOD_CHANNEL ** channel)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_PlaySound(system->_fmodSystem, channelIndex, sound, paused, channel);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::playDSP(FMOD_CHANNELINDEX channelIndex,
					FMOD_DSP * dsp,
					bool paused,
					FMOD_CHANNEL ** channel)
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_PlayDSP(system->_fmodSystem, channelIndex, dsp, paused, channel);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT UAudioSystem::update()
{
	FMOD_RESULT result = FMOD_OK;
    UAudioSystem* system = UAudioSystem::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = FMOD_System_Update(system->_fmodSystem);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}
// END - FMOD System encapsulated methods
