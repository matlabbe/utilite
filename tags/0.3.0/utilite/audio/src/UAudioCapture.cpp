/*********************************
Fonction de la classe UAudioRecorder
*********************************/
#include "utilite/UAudioCapture.h"
#include "UAudioSystem.h"
#include <utilite/ULogger.h>
#include <string.h>

/*********************************
Fonction constructeur de UAudioRecorder
*********************************/
UAudioCapture::UAudioCapture(int fs,
		 	 	   int frameLength,
		 	 	   int bytesPerSample,
		 	 	   int channels) :
    _fs(fs),
    _frameLength(frameLength),
    _bytesPerSample(bytesPerSample),
    _channels(channels),
    _nextFrameToGet(0)
{
	UAudioSystem::acquire();
    UDEBUG("");
}

/*********************************
Fonction destructeur de UAudioRecorder
*********************************/
UAudioCapture::~UAudioCapture()
{
    UDEBUG("");
    this->join(true);
    _getTrameSemaphore.release();
    UAudioSystem::release();
}

bool UAudioCapture::getNextFrame(std::vector<char> & frame, int &frameId)
{
    frameId = _nextFrameToGet++;
    return getFrame(frame, frameId);
}

bool UAudioCapture::getNextFrame(std::vector<char> & frame, bool removeOldFrames)
{
    int frameId = _nextFrameToGet;
    bool result =  getFrame(frame, frameId);
    if(result)
    {
    	++_nextFrameToGet;
		if(removeOldFrames)
		{
			removeFrames(0, _nextFrameToGet);
			_nextFrameToGet = 0;
		}
    }
    return result;
}

int UAudioCapture::getNextFrameToGet()
{
    return _nextFrameToGet;
}

/*********************************
Fonction getTrame
Permet de retourner la trame demand�e
*********************************/
bool UAudioCapture::getFrame(std::vector<char> & frame, int frameId)
{
    return getMultiFrame(frame, frameId, frameId);
}

/*********************************
Fonction getMultiTrame
Permet de retourner les trames demand�es dans une seule trame
*********************************/
bool UAudioCapture::getMultiFrame(std::vector<char> & frame, int frameIdBeg, int frameIdEnd)
{
	while(this->isCreating())
	{
		uSleep(1);
	}

	frame.clear();
	int samplesSize = samples();

    while (!( samplesSize  &&  (frameIdBeg <= frameIdEnd)  &&  (frameIdEnd < (samplesSize / (_frameLength * _bytesPerSample * _channels))) ) )
    {
        if(this->isKilled() || this->isIdle())
        {
        	UDEBUG("");
            return false;
        }

        // On attend qu'une trame soit pr�te
        _getTrameSemaphore.acquire();
        
        // On recalcule le total de la longueur des trames
        // TODO : � optimiser, le s�maphore devrait seulement "POST-er" lorsque il y a assez de trames d'enregistr�es...
        //        seulement un break; devrait �tre utilis�...
        samplesSize = samples();
    }
    
    unsigned int posBeg = frameIdBeg * _frameLength * _bytesPerSample * _channels;

    frame.resize(_frameLength * _bytesPerSample * _channels);
    
    _samplesMutex.lock();
    {             
    	memcpy(frame.data(), &_samples[posBeg], frame.size());
    }
    _samplesMutex.unlock();
    return true;
}

/*********************************
Fonction getNbTrame
Permet de tourner le nombre de trames disponibles
*********************************/
int UAudioCapture::getNumFrames()
{
    return samples()/_frameLength;
}

int UAudioCapture::samples()
{
    int size = 0;
    _samplesMutex.lock();
    {             
        size = (int)_samples.size();
    }
    _samplesMutex.unlock();
    return size;
}

void UAudioCapture::pushBackSamples(void * data, int dataLengthInBytes)
{
    _samplesMutex.lock();
    {
        int oldSize = (int)_samples.size();
        _samples.resize(oldSize + dataLengthInBytes);

        memcpy(&_samples[oldSize], data, dataLengthInBytes);

        if(dataLengthInBytes && _getTrameSemaphore.value() <= 0)
        {
        	_getTrameSemaphore.release();
        }
    }
    _samplesMutex.unlock();
}

void UAudioCapture::removeFrames(int frameIdBeg, int frameIdEnd)
{
    _samplesMutex.lock();
    {
        int posBeg = frameIdBeg * _frameLength * _bytesPerSample * _channels;
        int posEnd = frameIdEnd * _frameLength * _bytesPerSample * _channels;
        if(frameIdBeg <= frameIdEnd && posEnd <= (int)_samples.size())
        {
            // erase the elements:
             _samples.erase (_samples.begin() + posBeg, _samples.begin() + posEnd);
        }
    }
    _samplesMutex.unlock();
}

void UAudioCapture::removeSamples(int sampleByteBeg, int sampleByteEnd)
{
    _samplesMutex.lock();
    {
        int posBeg = sampleByteBeg;
        int posEnd = sampleByteEnd;
        if(posBeg <= posEnd && posEnd <= (int)_samples.size())
        {
            // erase the elements:
             _samples.erase (_samples.begin() + posBeg, _samples.begin() + posEnd);
        }
    }
    _samplesMutex.unlock();
}

void UAudioCapture::mainLoopEnd()
{
	_getTrameSemaphore.release();
}
