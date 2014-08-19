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

#ifndef AUDIOWIDGET_H_
#define AUDIOWIDGET_H_

#include "utilite/UtiLiteQtExp.h" // DLL export/import defines

#include <QtGui/QWidget>

class QGraphicsView;
class QGraphicsPixmapItem;
class UOrientableLabel;
class QAction;
class QMenu;
class QImage;
class QLabel;
class QGridLayout;

/**
 * USpectrogram is a QWidget to show a spectrogram. Add
 * frames (squared magnitude values) incrementally using Qt signals/slots.
 * @image html USpectrogram.png "Chirp"
 *
 * Example:
 * @code
 * #include "utilite/USpectrogram.h"
 * #include <QApplication>
 *
 * int main(int argc, char * argv[])
 * {
 *  QApplication app(argc, argv);
 *	USpectrogram spectrogram;
 *
 *	// Get audio frames and compute the FFTs,
 *	// compute the squared magnitude (im*im + re*re) of the FFTs, then
 *	// add the frequency frames to the spectrogram.
 *	std::vector<float> frame1, frame2, frame3, frame4; //...
 *	spectrogram.push(frame1);
 *	spectrogram.push(frame2);
 *	spectrogram.push(frame3);
 *	spectrogram.push(frame4);
 *	spectrogram.push(frameXXXX);
 *	//...

 *	spectrogram.show();
 *	app.exec();
 * 	return 0;
 * }
 * @endcode
 *
 *
 */
class UTILITEQT_EXP USpectrogram : public QWidget
{
	Q_OBJECT;

public:
	/**
	 * Constructor 1.
	 */
	USpectrogram(QWidget * parent = 0);
	/**
	 * Constructor 2. Set the sampling frequency of the frames (used to show values in Hz).
	 */
	USpectrogram(int fs, QWidget * parent = 0);
	virtual ~USpectrogram();
	void setScaled(bool freqScaled, bool timeScaled);
	void setZoomed(bool zoomed);
	void setAxesSwitched(bool axesSwitched);
	void setOnlyLastFramesDrawn(bool onlyLastFramesDrawn);
	void setAllDataKept(bool allDataKept);
	void setHorizontalScrollBarValue(int value);
	void setVerticalScrollBarValue(int value);
	void setDBRelative(bool dBRelative);
	void setDBGain(float value);
	void setDBMin(float value);
	int samplingRate() const {return _fs;}
	bool isScaledTime() const;
	bool isScaledFreq() const;
	bool isAxesSwitched() const;
	bool isZoomed() const;
	bool isOnlyLastFramesDrawn() const;
	bool isAllDataKept() const;
	bool isDBRelative() const;
	/**
	 * Set all frames. If you want to incrementally add frames, use slot push().
	 */
	void setData(const QList<QVector<float> > & data);
	USpectrogram * clone() const;

public slots:
	/**
	 * Clear the spectrogram.
	 */
	void clear();
	/**
	 * Push a new frame. It must be the same size of the previous
	 * added frames. If not, a clear() is required.
	 */
	void push(const std::vector<float> & frame);
	/**
	 * For convenience, push a new frame. It must be the same size of the previous
	 * added frames. If not, a clear() is required.
	 */
	void push(const QVector<float> & frame);
	/**
	 * Set sampling rate (to display frequencies in Hz).
	 */
	void setSamplingRate(int fs);

protected:
	virtual void resizeEvent(QResizeEvent * event);
	virtual void showEvent(QShowEvent * event);
	virtual void mouseMoveEvent(QMouseEvent * event);
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	void setupUi();
	void updateView();
	void push(const float frame[], int frameLength);

private:
	QGraphicsView * _view;
	QList<QVector<QRgb> > _rgbData;
	QList<QVector<QRgb> > _rgbLogData;
	QList<QVector<float> > _data;
	QList<QVector<float> > _dataLog;
	int _fs;
	QGraphicsPixmapItem * _imageItem;
	UOrientableLabel * _xLabel;
	UOrientableLabel * _yLabel;
	bool _axesSwitched;
	QGridLayout * _gridLayout;
	int _nbSubOctave;
	int _minLogSample;
	float _dBGain;
	float _dBMin;

	QMenu * _menu;
	QAction * _aClear;
	QAction * _aOnlyLastFramesDrawn;
	QAction * _aAllDataKept;
	QAction * _aScaledFreq;
	QAction * _aScaledTime;
	QAction * _aZoom2x;
	QAction * _aSwitchAxes;
	QAction * _aOpenNewWindow;
	QAction * _aLogFrequency;
	QAction * _aSaveTo;
	QAction * _aCopyFrame;
	QAction * _aPlotFrame;
	QAction * _aDBRelative;
	QAction * _aDBMin;
	QAction * _aDBGain;
};

#endif /* AUDIOWIDGET_H_ */
