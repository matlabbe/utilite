/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef AUDIOWIDGET_H_
#define AUDIOWIDGET_H_

#include "utilite/UtiLiteExp.h" // DLL export/import defines

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
class UTILITE_EXP USpectrogram : public QWidget
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
	void setHorizontalScrollBarValue(int value);
	void setVerticalScrollBarValue(int value);
	void setDBGain(float value);
	void setDBMin(float value);
	int samplingRate() const {return _fs;}
	bool isScaledTime() const;
	bool isScaledFreq() const;
	bool isAxesSwitched() const;
	bool isZoomed() const;
	bool isOnlyLastFramesDrawn() const;
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
	QAction * _aScaledFreq;
	QAction * _aScaledTime;
	QAction * _aZoom2x;
	QAction * _aSwitchAxes;
	QAction * _aOpenNewWindow;
	QAction * _aLogFrequency;
	QAction * _aSaveTo;
	QAction * _aDBMin;
	QAction * _aDBGain;
};

#endif /* AUDIOWIDGET_H_ */
