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
class UPlotOrientableLabel;
class QAction;
class QMenu;
class QImage;
class QLabel;
class QGridLayout;

class UTILITE_EXP USpectrogram : public QWidget
{
	Q_OBJECT;

public:
	USpectrogram(QWidget * parent = 0);
	USpectrogram(int fs, QWidget * parent = 0);
	virtual ~USpectrogram();
	void setScaled(bool freqScaled, bool timeScaled);
	void setZoomed(bool zoomed);
	void setAxesSwitched(bool axesSwitched);
	void setOnlyLastFramesDrawn(bool onlyLastFramesDrawn);
	void setHorizontalScrollBarValue(int value);
	void setVerticalScrollBarValue(int value);
	int samplingRate() const {return _fs;}
	bool isScaledTime() const;
	bool isScaledFreq() const;
	bool isAxesSwitched() const;
	bool isZoomed() const;
	bool isOnlyLastFramesDrawn() const;
	void setData(const QList<QVector<float> > & data);
	void clear();
	USpectrogram * clone() const;

public slots:
	void push(const std::vector<float> & frame);
	void push(const QVector<float> & frame);
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
	UPlotOrientableLabel * _xLabel;
	UPlotOrientableLabel * _yLabel;
	bool _axesSwitched;
	QGridLayout * _gridLayout;
	int _nbSubOctave;
	int _minLogSample;

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
};

#endif /* AUDIOWIDGET_H_ */
