#include "System.h"
#include <utilite/ULogger.h>

//=========================
//public:  
//=========================
void System::ERRCHECK(const FMOD_RESULT & result)
{
    if (result != FMOD_OK)
    {
        UERROR("FMOD error! (%d) %s\n", result, FMOD_ErrorString(result));
    }
}

void System::release()
{
	FMOD_RESULT result;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->close();        ERRCHECK(result);
        result = system->_fmodSystem->release();        ERRCHECK(result);
        system->_fmodSystem = 0;
        system->_fmodSystemMutex.unlock();
    }
}

//=========================
//private:
//=========================
System* System::instance()
{  
    static System system;
    if(system._fmodSystem == 0)
    {
        system._fmodSystemMutex.lock();
        system.init();
        system._fmodSystemMutex.unlock();
    }
    return &system;
}

System::System() : _fmodSystem(0)
{
    UDEBUG("");
    init();
}

System::~System()
{
    UDEBUG("");
    // Probleme ici lorsque l'application termine, _fmodSystem->close() plante...
    // Appeler à la place System::release() dans le main
    FMOD_RESULT result;
    if(_fmodSystem) {
    	_fmodSystemMutex.lock();
        result = _fmodSystem->close();        ERRCHECK(result);
        result = _fmodSystem->release();        ERRCHECK(result);
        _fmodSystem = 0;
        _fmodSystemMutex.unlock();
    }
}

void System::init()
{
    if(_fmodSystem) {
        return;
    }
    UDEBUG("");

    FMOD_RESULT result;

    //  Create a System object.
    result = FMOD::System_Create(&_fmodSystem);        ERRCHECK(result);

    //TODO : faire un check de version de FMOD
    unsigned int version;
    result = _fmodSystem->getVersion(&version);        ERRCHECK(result);
    if (version < FMOD_VERSION)    //Vérification de la version de la librairie FMOD du systmème
    {
        UFATAL("Erreur!  Vous utilisez une vieille version de FMOD %08x.  Ce programme requiere la version %08x", version, FMOD_VERSION);
    }


    //Set output device
    UDEBUG("Auto detecting the output device...");
    _fmodSystem->setOutput(FMOD_OUTPUTTYPE_AUTODETECT);        ERRCHECK(result);
    //_fmodSystem->setOutput(FMOD_OUTPUTTYPE_DSOUND);            ERRCHECK(result); // Doesn't work on Vista
    //_fmodSystem->setOutput(FMOD_OUTPUTTYPE_WASAPI);            ERRCHECK(result); // Vista only

    FMOD_OUTPUTTYPE output;
    _fmodSystem->getOutput(&output);

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


    result = _fmodSystem->init(32, FMOD_INIT_NORMAL, 0);        ERRCHECK(result);
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
FMOD_RESULT System::getRecordNumDrivers(int * numDrivers)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->getRecordNumDrivers(numDrivers);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::getRecordDriverInfo(int id,
                                char * name,
                                int namelen, 
                                FMOD_GUID* guid)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->getRecordDriverInfo(id, name, namelen, guid);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::isRecording(int driver, bool * isRecording)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->isRecording(driver, isRecording);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::recordStart(int driver,
						FMOD::Sound * sound,
						bool loop)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->recordStart(driver, sound, loop);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::recordStop(int driver)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->recordStop(driver);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::getRecordPosition(int driver, unsigned int * position)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->getRecordPosition(driver, position);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::createSound(const char * name_or_data,
                        FMOD_MODE mode,
                        FMOD_CREATESOUNDEXINFO * exinfo,
                        FMOD::Sound ** sound)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->createSound(name_or_data, mode, exinfo, sound);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::createStream(const char * name_or_data,
                                   FMOD_MODE mode,
                                   FMOD_CREATESOUNDEXINFO * exinfo,
                                   FMOD::Sound ** sound)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->createSound(name_or_data, mode, exinfo, sound);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}
FMOD_RESULT System::createDSPByType(FMOD_DSP_TYPE dspType,
                            FMOD::DSP ** dsp)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->createDSPByType(dspType, dsp);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::playSound(FMOD_CHANNELINDEX channelIndex,
						FMOD::Sound * sound,
						bool paused,
						FMOD::Channel ** channel)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->playSound(channelIndex, sound, paused, channel);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::playDSP(FMOD_CHANNELINDEX channelIndex,
					FMOD::DSP * dsp,
					bool paused,
					FMOD::Channel ** channel)
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->playDSP(channelIndex, dsp, paused, channel);
        system->_fmodSystemMutex.unlock();
    }
    return result;
}

FMOD_RESULT System::update()
{
	FMOD_RESULT result = FMOD_OK;
    System* system = System::instance();
    if(system->_fmodSystem) {
        system->_fmodSystemMutex.lock();
        result = system->_fmodSystem->update();
        system->_fmodSystemMutex.unlock();
    }
    return result;
}
// END - FMOD System encapsulated methods
