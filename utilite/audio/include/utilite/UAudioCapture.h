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

#ifndef UAUDIORECORDER_H
#define UAUDIORECORDER_H

#include "utilite/UtiLiteAudioExp.h" // DLL export/import defines

#include <utilite/USemaphore.h>
#include <utilite/UMutex.h>
#include <utilite/UThreadNode.h>

#include <vector>

/**
 * This is the abstract class for all
 * recorders. A recorder is used to get fixed 
 * frame length from a stream. It is configured 
 * with the length of the frame wanted. 
 * The main methods are Start, 
 * stop, and getNextFrame. See UAudioCaptureMic and
 * UAudioCaptureFile for implementation. To extract samples in
 * frequency domain, see UAudioCaptureFFT.
 *
 * \n Example : 
 * @code
 *    UAudioCapture* recorder = <<SeeInheritedClasses>>;
 *    Frame frame;
 *    unsigned int frameId = 0;
 *    recorder->start();
 *    while(recorder->getNextFrame(frame, frameId))
 *    { 
 *      ...process the frame...
 *      ... break when a condition is reached...
 *    }
 *    recorder->stop();
 *    delete recorder;
 * @endcode
 *
 * @see UAudioCapture::start
 * @see UAudioCapture::stop
 * @see UAudioCapture::getNextFrame
 *
 */
class UTILITEAUDIO_EXP UAudioCapture : public UThread {
public:
    virtual ~UAudioCapture();

    virtual bool init()
    {
    	_samplesMutex.lock();
		_samples.clear();
		_samplesMutex.unlock();
		return true;
    }
    virtual void close() {}

    void stop() {join(true);} // for convenience

    /**
     * Get the next frame from the stream. The caller will wait 
     * until the length of the frame required is reached. The 
     * parameter removeOldFrames can be used to clean the 
     * memory after each call. When removeOldFrames = true, 
     * the frame id is reseted.
     * @param frame the result frame
     * @param removeOldFrames true to remove old frames
     * @return true on success, this method failed when the 
     *         end of the stream is reached.
     */
    bool getNextFrame(std::vector<char> & frame,
                      bool removeOldFrames = false);

    /**
     * Get the next frame from the stream. The caller will wait 
     * until the length of the frame required is reached.
     * @param frame the result frame
     * @param frameId the id of the frame
     * @return true on success, this method failed when the 
     *         end of the stream is reached.
     * @warning if getNextFrame with removeOldFrames=true 
     *          is called, the id is reseted.
     */
    bool getNextFrame(std::vector<char> & frame,
                      int &frameId);

    /**
     * Get the multiple frames from the stream. The caller 
     * will wait until enough data is recorded.
     * @param frame the result frame
     * @param frameIdBeg the id of the begin frame
     * @param frameIdEnd the id of the end frame
     * @return true on success, this method failed when the 
     *         end of the stream is reached.
     * @warning if getNextFrame with removeOldFrames=true 
     *          is called, the id is reseted.
     */
    bool getMultiFrame(std::vector<char> & frame,
                      int frameIdBeg,
                      int frameIdEnd);
    
    /**
     * Get a specific frame from the stream. If the id is 
     * higher than the last frame recorded, the caller 
     * will wait until enough data is recorded.
     * @param frame the result frame
     * @param frameId the id of the frame
     * @return true on success, this method failed when the 
     *         end of the stream is reached.
     * @warning if getNextFrame with removeOldFrames=true 
     *          is called, the id is reseted.
     */
    bool getFrame(std::vector<char> & frame, int frameId);

    /**
     * Used to clean the memory.
     * @param frameIdBeg the id of the begin frame
     * @param frameIdEnd the id of the end frame
     * @warning if getNextFrame with removeOldFrames=true 
     *          is called, the id is reseted.
     */
    void removeFrames(int frameIdBeg,
                      int frameIdEnd);

    /**
	 * Used to clean the memory.
	 * @param sampleByteBeg sample byte begin
	 * @param sampleByteEnd sample byte end
	 */
	void removeSamples(int sampleByteBeg,
					   int sampleByteEnd);

    int getNumFrames();
    int getNextFrameToGet();

    int samples();

    int fs() const {return _fs;}
    int frameLength() const {return _frameLength;}
    int bytesPerSample() const {return _bytesPerSample;}
    int channels() const {return _channels;}

    void setFrameLength(int frameLength) {_frameLength = frameLength;}

protected:
    UAudioCapture(int fs = 44100,
    		 int frameLength = 1024,
    		 int bytesPerSample = 2,
    		 int channels = 1);

    virtual void mainLoopEnd();
    void pushBackSamples(void * data, int dataLengthInBytes);

    void setFs(int fs) {_fs = fs;}
	void setBytesPerSample(int bytesPerSample) {_bytesPerSample = bytesPerSample;}
	void setChannels(int channels) {_channels = channels;}

private:
    UMutex _samplesMutex;
    std::vector<char> _samples;

    int _fs;
    int _frameLength;
    int _bytesPerSample;
    int _channels;

    int _nextFrameToGet;
    USemaphore _getTrameSemaphore;

};

#endif
