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

#include "utilite/USpectrogram.h"
#include "utilite/UPlot.h"

#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QGridLayout>
#include <QtGui/QAction>
#include <QtGui/QMenu>
#include <QtGui/QContextMenuEvent>
#include <QtGui/QToolTip>
#include <QtGui/QFileDialog>
#include <QtGui/QScrollBar>
#include <QtGui/QApplication>
#include <QtGui/QInputDialog>
#include <QtGui/QClipboard>
#include <QtCore/QDir>
#include <QtCore/QTimer>
#include <utilite/ULogger.h>
#include <utilite/UMath.h>
#include <cmath>

#define PRINT_DEBUG 0

#define XLABEL_DEFAULT "0 -- Frequency --> 2*pi rad/sample"
#define YLABEL_DEFAULT "0 -- Time -->"
#define PI 3.14159

#if defined(WIN32) && !defined(__MINGW32__)
// Calculates log2 of number.  
double log2( double n )  
{  
    // log(n)/log(2) is log2.  
    return log( n ) / log( 2.0 );  
}
#endif

class View : public QGraphicsView
{
public:
	View(QWidget * parent = 0) :
		QGraphicsView(parent)
	{
		this->setMouseTracking(true);
	}
	virtual ~View() {}

protected:
	virtual void mouseMoveEvent(QMouseEvent * event)
	{
		event->ignore();
	}
};

USpectrogram::USpectrogram(QWidget * parent) :
	QWidget(parent),
	_fs(0),
	_dBGain(0),
	_dBMin(-60)
{
	this->setupUi();
}

USpectrogram::USpectrogram(int fs, QWidget * parent) :
	QWidget(parent),
	_fs(fs),
	_dBGain(0),
	_dBMin(-60)
{
	this->setupUi();
}

void USpectrogram::setupUi()
{
	qRegisterMetaType<std::vector<float> >("std::vector<float>");
	this->setMouseTracking(true);
	setWindowTitle("Squared magnitude in dB (red = 0 dB)");

	_view = new View(this);
	_view->setScene(new QGraphicsScene(this));
	_view->scene()->setBackgroundBrush(QBrush(Qt::black));
	_imageItem = new QGraphicsPixmapItem();
	_view->scene()->addItem(_imageItem);

	_axesSwitched = true;
	if(_fs<=0)
	{
		_xLabel = new UOrientableLabel(XLABEL_DEFAULT, Qt::Vertical, this);
	}
	else
	{
		_xLabel = new UOrientableLabel(QString("0 -- Frequency --> %1 Hz").arg(_fs/2), Qt::Vertical, this);
	}
	_yLabel = new UOrientableLabel(YLABEL_DEFAULT, Qt::Horizontal, this);
	_xLabel->setAlignment(Qt::AlignCenter);
	_yLabel->setAlignment(Qt::AlignCenter);

	//Layout
	_gridLayout = new QGridLayout(this);
	_gridLayout->setContentsMargins(0,0,0,0);
	_gridLayout->setVerticalSpacing(0);
	_gridLayout->setHorizontalSpacing(0);
	_gridLayout->addWidget(_view, 0, 1);
	_gridLayout->addWidget(_xLabel, 0, 0);
	_gridLayout->addWidget(_yLabel, 1, 1);

	//Menu
	_menu = new QMenu(this);
	_aOnlyLastFramesDrawn = _menu->addAction("Only last frames drawn");
	_aOnlyLastFramesDrawn->setCheckable(true);
	_aOnlyLastFramesDrawn->setChecked(false);
	_aOnlyLastFramesDrawn->setEnabled(false);
	_aAllDataKept = _menu->addAction("All data kept");
	_aAllDataKept->setCheckable(true);
	_aAllDataKept->setChecked(false);
	QMenu * zoomMenu = _menu->addMenu("Zoom");
	_aZoom2x = zoomMenu->addAction("2x");
	_aZoom2x->setCheckable(true);
	_aZoom2x->setChecked(false);
	_aSwitchAxes = _menu->addAction("Switch axes");
	_aOpenNewWindow = _menu->addAction("Open in a new window");
	_aLogFrequency = _menu->addAction("Log frequency");
	_aLogFrequency->setCheckable(true);
	_aLogFrequency->setChecked(false);
	_aSaveTo = _menu->addAction("Save to...");
	_aCopyFrame = _menu->addAction("Copy spectrum to clipboard");
	_aPlotFrame = _menu->addAction("Plot spectrum");

	_menu->addSeparator();

	_aDBRelative = _menu->addAction(tr("Show dB relative to current max"));
	_aDBRelative->setCheckable(true);
	_aDBRelative->setChecked(true);
	_aDBMin = _menu->addAction(tr("Min dB (%1 dB)").arg(_dBMin));
	_aDBGain = _menu->addAction(tr("Gain dB (%1 dB)").arg(_dBGain));

	_menu->addSeparator();

	_aScaledFreq = _menu->addAction("Frequency scaled");
	_aScaledTime = _menu->addAction("Time scaled");
	_aScaledFreq->setCheckable(true);
	_aScaledTime->setCheckable(true);
	this->setScaled(true, false);
	this->setOnlyLastFramesDrawn(true);

	_nbSubOctave = 16;
	_minLogSample = 1; // ignore DC

	_menu->addSeparator();

	_aClear = _menu->addAction("Clear");
}

USpectrogram::~USpectrogram()
{
}

void USpectrogram::setSamplingRate(int fs)
{
	if(_fs != fs)
	{
		_fs = fs;
		if(_fs <= 0)
		{
			_xLabel->setText(XLABEL_DEFAULT);
			_yLabel->setText(YLABEL_DEFAULT);
		}
		else
		{
			_xLabel->setText(QString("0 -- Frequency --> %1 Hz").arg(_fs/2));
		}
	}
}

void USpectrogram::setDBGain(float value)
{
	_dBGain = value;
	_aDBGain->setText(tr("Gain dB (%1 dB)").arg(_dBGain));
}

void USpectrogram::setDBMin(float value)
{
	_dBMin = value;
	_aDBMin->setText(tr("Min dB (%1 dB)").arg(_dBMin));
}

void USpectrogram::push(const std::vector<float> & frame)
{
	this->push(frame.data(), frame.size());
}

void USpectrogram::push(const QVector<float> & frame)
{
	this->push(frame.data(), frame.size());
}

void USpectrogram::push(const float frame[], int frameLength)
{
	if(!_rgbData.isEmpty() && frameLength != _rgbData.first().size())
	{
		UWARN("Frames must all have the same size (%d vs %d), the spectrogram is cleared before adding the new frame...", _rgbData.first().size(), frameLength);
		this->clear();
	}
	else if(!frameLength || !frame)
	{
		UERROR("frame is empty");
		return;
	}

	int val; // max 240 // H=blue
	float max = frameLength*frameLength;
	if(_aDBRelative->isChecked())
	{
		max = uMax(frame, frameLength);
	}
	float minDb = _dBMin;//fabs(10*std::log10(min/max));
	float gain = _dBGain;

	float frameMin = 0.0f;
	float frameMax = 0.0f;
	uMinMax(frame, frameLength, frameMin, frameMax);
	bool isInDB = false;
	if(frameMin < 0 && frameMax <= 0)
	{
		isInDB = true;
	}
#if PRINT_DEBUG
	UDEBUG("frameLength = %d, max=%f, gain=%f, minDb=%f, frameMin=%f, frameMax=%f, isInDB=%d", frameLength, max, gain, -minDb, frameMin, frameMax, isInDB?1:0);
#endif
	QVector<QRgb> v(frameLength);

	// Log stuff
	QVector<QRgb> vLog(log2(frameLength) * _nbSubOctave + 1);
	QVector<float> dataLog(vLog.size());
#if PRINT_DEBUG
	UDEBUG("is=%d, js=%d", frameLength, vLog.size());
#endif

	int octaveStart = _minLogSample;
	int avg = 0;
	float avgMag = 0;
	int count = 0;
	int j = 0;
	int k = 0;
	float subOctaveWidth = float(octaveStart)/float(_nbSubOctave);
	for(int i=0; i<v.size(); ++i)
	{
		float frameVal = minDb;
		if(isInDB)
		{
			frameVal = frame[i];
			if(frame[i] == 0)
			{
				frameVal = minDb;
			}
		}
		else
		{
			if(frame[i]>0)
			{
				frameVal = 10*std::log10(frame[i]/max) + gain;
			}
			if(frameVal < minDb)
			{
				frameVal = minDb;
			}
		}
		val = (frameVal-gain)*240.0/(minDb-gain);
		if(val < 0)
		{
			val = 0; // red
		}
		else if(val>240)
		{
			val = 240; // blue
		}
		v[i] = QColor::fromHsv(val, 255, 128).rgb();
		if(i>=octaveStart)
		{
			// log indexes = 1 2 4 8 16 32 64 128 256 512
			// sub2-log indexes 1 1.5 2 3 4 6 8 12 16 24 32 48 64 96 128 192 256 384 512
			// sub3-log indexes 1 1.3 1.6 2...
			avgMag += frame[i];
			avg += val;
			++count;
			if(i >= octaveStart + int(float(k+1)*subOctaveWidth) - 1)
			{
				avgMag/=float(count);
				avg/=count;
				//UDEBUG("i=%d, j=%d, k=%d octaveStart=%d (suboct=%d) -> hue=%d, count=%d, octWidth=%f", i, j, k, octaveStart, octaveStart + int(float(k)*subOctaveWidth), avg, count, subOctaveWidth);
				dataLog[j] = avgMag;
				vLog[j++] = QColor::fromHsv(avg, 255, 128).rgb();
				while(i > octaveStart + int(float(k+1)*subOctaveWidth) - 1)
				{
					++k;
					//UDEBUG("i=%d, j=%d, k=%d octaveStart=%d (suboct=%d) -> hue=%d, count=%d, octWidth=%f", i, j, k, octaveStart, octaveStart + int(float(k)*subOctaveWidth), avg, count, subOctaveWidth);
					dataLog[j] = avgMag;
					vLog[j++] = QColor::fromHsv(avg, 255, 128).rgb();
				}
				avgMag = 0;
				avg = 0;
				count = 0;
				if(++k >= _nbSubOctave)
				{
					k = 0;
					octaveStart *= 2;
					subOctaveWidth = float(octaveStart)/float(_nbSubOctave);

				}
			}
		}
	}
	if(dataLog.size() > j)
	{
		dataLog.resize(j);
		vLog.resize(j);
	}
#if PRINT_DEBUG
	UDEBUG("j=%d, octaveStart=%d, subOctaveWidth=%f", j, octaveStart, subOctaveWidth);
#endif

	_rgbData.push_front(v);
	_rgbLogData.push_front(vLog);
	QVector<float> dataCopy(frameLength);
	memcpy(dataCopy.data(), frame, frameLength*sizeof(float));
	_data.push_front(dataCopy);
	_dataLog.push_front(dataLog);

	if(_fs)
	{
		_yLabel->setText(QString("0 -- Time --> %1 s (%2 frames)").arg(float(_rgbData.size()) * float(_rgbData.first().size()) / float(_fs/2), 0 ,'f', 1).arg(_rgbData.size()));
	}
	else
	{
		_yLabel->setText(QString("0 -- Time --> %1 frames").arg(_rgbData.size()));
	}

	this->updateView();
}

void USpectrogram::resizeEvent(QResizeEvent * event)
{
	this->updateView();
}

void USpectrogram::showEvent(QShowEvent * event)
{
	this->updateView();
}

void USpectrogram::mouseMoveEvent(QMouseEvent * event)
{
	QPoint pos = _view->mapFromGlobal(event->globalPos());
	pos.setX(pos.x()-3);
	pos.setY(pos.y()-3);
	QPointF posf = _view->mapToScene(pos);
	pos.setX(posf.x());
	pos.setY(posf.y());
	QList<QVector<float> > * data = &_data;
	if(_aLogFrequency->isChecked())
	{
		data = &_dataLog;
	}
	if(data->size() &&
			pos.x()>=0 && pos.x()<data->front().size() &&
			pos.y()>=0 && pos.y()<data->size())
	{
		//Compute dB
		float max = _data.at(pos.y()).size() * _data.at(pos.y()).size();
		if(_aDBRelative->isChecked())
		{
			max = uMax(_data.at(pos.y()).data(), _data.at(pos.y()).size());
		}
		float dB = data->at(pos.y()).at(pos.x());
		if(dB > 0)
		{
			dB = 10*std::log10(dB/max);
		}
		else if(dB==0)
		{
			dB=-999;
		}

		float posx = pos.x();
		if(_aLogFrequency->isChecked())
		{
#if PRINT_DEBUG
			UDEBUG("posx=%f logData=%d", posx, data->front().size());
#endif
			posx = std::pow(2.0f, float(posx-1)/float(_nbSubOctave));
#if PRINT_DEBUG
			UDEBUG("posx=%f", posx);
#endif
			posx = posx * float(_data.front().size()) / std::pow(2.0f, float(data->front().size()-1)/float(_nbSubOctave)) + float(_minLogSample);
		}
		if(_fs)
		{
			QToolTip::showText(event->globalPos(),
					QString("Frame %1 (%2 s), Frequency %3 Hz (sample %6), Squared Magnitude %4 (%5 dB)")
					.arg(data->size()-pos.y())
					.arg(float(data->size()-pos.y()) * float(_data.first().size()) / float(_fs/2), 0 ,'f', 3)
					.arg(posx*float(_fs/2)/float(_data.front().size()), 0, 'f', 0)
					.arg(data->at(pos.y()).at(pos.x())>0?data->at(pos.y()).at(pos.x()):-1)
					.arg(dB, 0, 'f', 1)
					.arg(int(posx)));
		}
		else
		{
			QToolTip::showText(event->globalPos(),
					QString("Frame %1, Frequency %2 rad/sample (sample %5), Squared Magnitude %3 (%4 dB)")
					.arg(data->size()-pos.y())
					.arg(posx*2.0f*3.1459f/float(_data.front().size()), 0, 'f', 2)
					.arg(data->at(pos.y()).at(pos.x())>0?data->at(pos.y()).at(pos.x()):-1)
					.arg(dB, 0, 'f', 1)
					.arg(int(posx)));
		}
	}
	else
	{
		QToolTip::hideText();
	}
}

void USpectrogram::clear()
{
	_rgbData.clear();
	_rgbLogData.clear();
	_data.clear();
	_dataLog.clear();
	_view->scene()->removeItem(_imageItem);
	_imageItem->setPixmap(QPixmap());
	delete _view->scene();
	_view->setScene(new QGraphicsScene(this));
	_view->scene()->setBackgroundBrush(QBrush(Qt::black));
	_view->scene()->addItem(_imageItem);
	_xLabel->setText(XLABEL_DEFAULT);
	if(_fs>0)
	{
		_xLabel->setText(QString("0 -- Frequency --> %1 Hz").arg(_fs/2));
	}
	_yLabel->setText(YLABEL_DEFAULT);
}

void USpectrogram::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * a = _menu->exec(event->globalPos());
	if(a)
	{
		if(a == _aClear)
		{
			clear();
		}
		else if(a == _aOnlyLastFramesDrawn)
		{
			this->setOnlyLastFramesDrawn(_aOnlyLastFramesDrawn->isChecked());
		}
		else if(a == _aAllDataKept)
		{
			this->setAllDataKept(_aAllDataKept->isChecked());
		}
		else if(a == _aZoom2x)
		{
			this->setZoomed(_aZoom2x->isChecked());
		}
		else if(a == _aScaledFreq || a == _aScaledTime)
		{
			this->setScaled(_aScaledFreq->isChecked(), _aScaledTime->isChecked());
		}
		else if(a == _aSaveTo)
		{
			QString text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), (QDir::homePath() + "/") + QString("spectrogram") + ".png", "*.png *.bmp *.jpg");
			if(!text.isEmpty())
			{
				QImage image(_view->scene()->width(), _view->scene()->height(), QImage::Format_RGB32);
				QPainter painter(&image);
				_view->scene()->render(&painter);
				image.save(text);
			}
		}
		else if(a == _aCopyFrame || a == _aPlotFrame)
		{
			QPoint pos = _view->mapFromGlobal(event->globalPos());
			pos.setX(pos.x()-3);
			pos.setY(pos.y()-3);
			QPointF posf = _view->mapToScene(pos);
			pos.setX(posf.x());
			pos.setY(posf.y());
			QList<QVector<float> > * data = &_data;
			if(_aLogFrequency->isChecked())
			{
				data = &_dataLog;
			}
			if(data->size() &&
					pos.x()>=0 && pos.x()<data->front().size() &&
					pos.y()>=0 && pos.y()<data->size())
			{
				if(a == _aPlotFrame)
				{
					UPlot * plot = new UPlot(this);
					plot->setWindowFlags(Qt::Window);
					plot->setAttribute(Qt::WA_DeleteOnClose, true);
					plot->setGraphicsView(true);
					if(_fs)
					{
						plot->setWindowTitle(QString("Frame %1 (%2 s)")
									.arg(data->size()-pos.y())
									.arg(float(data->size()-pos.y()) * float(_data.first().size()) / float(_fs/2), 0 ,'f', 3));
					}
					else
					{
						plot->setWindowTitle(QString("Frame %1").arg(data->size()-pos.y()));
					}
					UPlotCurve * curve = plot->addCurve(QString("%1Spectrum").arg(_aLogFrequency->isChecked()?"Log ":""));
					UPlotCurve * curveDB = plot->addCurve(QString("%1Spectrum (dB)").arg(_aLogFrequency->isChecked()?"Log ":""));

					curve->setData(data->at(pos.y()));

					// transform to dB
					const QVector<float> & raw = data->at(pos.y());
					float max = uMax(raw.data(), raw.size());
					for(int i=0; i<raw.size(); ++i)
					{
						float val = 0.0f;
						if(max)
						{
							if(raw[i])
							{
								val = 10*std::log10(raw[i]/max);// transform to dB
							}
							else
							{
								val = _dBMin;
							}
							if(val < _dBMin)
							{
								val = _dBMin;
							}
						}
						else
						{
							val = 0.0f;
						}
						curveDB->addValue(val);
					}
					plot->show();
				}
				else // _aCopyFrame
				{
					QString text;
					for(int i=0; i<data->at(pos.y()).size(); ++i)
					{
						text.append(QString::number(data->at(pos.y()).at(i)));
						if(i+1<data->at(pos.y()).size())
						{
							text.append(' ');
						}
					}
					QClipboard * clipboard = QApplication::clipboard();
					clipboard->setText(text);
				}
			}
		}
		else if(a == _aSwitchAxes)
		{
			this->setAxesSwitched(!_axesSwitched);
		}
		else if(a==_aOpenNewWindow)
		{
			USpectrogram * newWindow = this->clone();
			newWindow->setAttribute(Qt::WA_DeleteOnClose, true);
			newWindow->show();
			newWindow->setHorizontalScrollBarValue(_view->horizontalScrollBar()->value());
			newWindow->setVerticalScrollBarValue(_view->verticalScrollBar()->value());
		}
		else if(a == _aLogFrequency)
		{
			_view->scene()->removeItem(_imageItem);
			delete _view->scene();
			_view->setScene(new QGraphicsScene(this));
			_view->scene()->setBackgroundBrush(QBrush(Qt::black));
			_view->scene()->addItem(_imageItem);
			this->updateView();
		}
		else if(a == _aDBRelative)
		{
			this->setDBRelative(_aDBRelative->isChecked());
		}
		else if(a == _aDBGain)
		{
			bool ok = false;
			float val = QInputDialog::getDouble(this, tr("New dB gain"), tr("Value"), _dBGain, 0, 100, 0, &ok);
			if(ok)
			{
				this->setDBGain(val);
			}
		}
		else if(a == _aDBMin)
		{
			bool ok = false;
			float val = QInputDialog::getDouble(this, tr("New dB min"), tr("Value"), _dBMin, -100, 0, 0, &ok);
			if(ok)
			{
				this->setDBMin(val);
			}
		}
	}
}

void USpectrogram::setScaled(bool freqScaled, bool timeScaled)
{
	_aScaledFreq->setChecked(freqScaled);
	_aScaledTime->setChecked(timeScaled);
	updateView();
}

void USpectrogram::setAxesSwitched(bool axesSwitched)
{
	if(_axesSwitched != axesSwitched)
	{
		_axesSwitched = axesSwitched;
		if(_axesSwitched)
		{
			_gridLayout->removeWidget(_xLabel);
			_gridLayout->removeWidget(_yLabel);
			_xLabel->setOrientation(Qt::Vertical);
			_yLabel->setOrientation(Qt::Horizontal);
			_gridLayout->addWidget(_yLabel, 1, 1);
			_gridLayout->addWidget(_xLabel, 0, 0);
		}
		else
		{
			_gridLayout->removeWidget(_xLabel);
			_gridLayout->removeWidget(_yLabel);
			_xLabel->setOrientation(Qt::Horizontal);
			_yLabel->setOrientation(Qt::Vertical);
			_gridLayout->addWidget(_xLabel, 1, 1);
			_gridLayout->addWidget(_yLabel, 0, 0);
		}
		this->updateView();
	}
}

void USpectrogram::setZoomed(bool zoomed)
{
	_aZoom2x->setChecked(zoomed);
	updateView();
}

void USpectrogram::setOnlyLastFramesDrawn(bool onlyLastFramesDrawn)
{
	_aOnlyLastFramesDrawn->setChecked(onlyLastFramesDrawn);
	this->updateView();
}

void USpectrogram::setAllDataKept(bool allDataKept)
{
	_aOnlyLastFramesDrawn->setEnabled(allDataKept);
	_aAllDataKept->setChecked(allDataKept);
	this->updateView();
}

void USpectrogram::setHorizontalScrollBarValue(int value)
{
	_view->horizontalScrollBar()->setValue(value);
}

void USpectrogram::setVerticalScrollBarValue(int value)
{
	_view->verticalScrollBar()->setValue(value);
}

void USpectrogram::setDBRelative(bool dBRelative)
{
	_aDBRelative->setChecked(dBRelative);
}

bool USpectrogram::isScaledFreq() const
{
	return _aScaledFreq->isChecked();
}

bool USpectrogram::isScaledTime() const
{
	return _aScaledTime->isChecked();
}

bool USpectrogram::isAxesSwitched() const
{
	return _axesSwitched;
}

bool USpectrogram::isZoomed() const
{
	return _aZoom2x->isChecked();
}

bool USpectrogram::isOnlyLastFramesDrawn() const
{
	return _aOnlyLastFramesDrawn->isChecked();
}

bool USpectrogram::isAllDataKept() const
{
	return _aAllDataKept->isChecked();
}

bool USpectrogram::isDBRelative() const
{
	return _aDBRelative->isChecked();
}

USpectrogram * USpectrogram::clone() const
{
	USpectrogram * newWidget = new USpectrogram(_fs);
	newWidget->setScaled(this->isScaledFreq(), this->isScaledTime());
	newWidget->setAxesSwitched(this->isAxesSwitched());
	newWidget->setZoomed(this->isZoomed());
	newWidget->setGeometry(this->geometry());
	newWidget->setOnlyLastFramesDrawn(this->isOnlyLastFramesDrawn());
	newWidget->setData(_data);
	return newWidget;
}

void USpectrogram::setData(const QList<QVector<float> > & data)
{
	this->clear();
	bool wasVisible = false;
	if(isVisible())
	{
		// Disable updateView in push
		wasVisible = true;
		this->setVisible(false);
	}

	for(int i=data.size()-1; i>=0; --i)
	{
		this->push(data.at(i));
	}

	if(wasVisible)
	{
		this->setVisible(true);
	}
}

void USpectrogram::updateView()
{
	if(isVisible())
	{
		if(_rgbData.size())
		{
			QImage img;
			int lastFramesCount = _view->height();
			if(_axesSwitched)
			{
				lastFramesCount = _view->width();
				if(_view->verticalScrollBar()->isVisible())
				{
					lastFramesCount -= _view->verticalScrollBar()->width()+2;
				}
			}
			else if(_view->horizontalScrollBar()->isVisible())
			{
				lastFramesCount -= _view->horizontalScrollBar()->height()+2;
			}
			if(_aZoom2x->isChecked() && !_aScaledTime->isChecked())
			{
				lastFramesCount/=2;
			}

			int freqCount = _rgbData.front().size();
			if(_aLogFrequency->isChecked())
			{
				freqCount = _rgbLogData.front().size();
			}

			if(!_aAllDataKept->isChecked() &&
				_rgbData.size() >= lastFramesCount-2)
			{
				_rgbData.erase(_rgbData.begin() + lastFramesCount-2, _rgbData.end());
				_rgbLogData.erase(_rgbLogData.begin() + lastFramesCount-2, _rgbLogData.end());
				_data.erase(_data.begin() + lastFramesCount-2, _data.end());
				_dataLog.erase(_dataLog.begin() + lastFramesCount-2, _dataLog.end());
			}

			if(_aOnlyLastFramesDrawn->isChecked() &&
				_rgbData.size() >= lastFramesCount-2)
			{
				img = QImage(freqCount, lastFramesCount-2, QImage::Format_RGB32);
			}
			else
			{
				img = QImage(freqCount, _rgbData.size(), QImage::Format_RGB32);
			}

#if PRINT_DEBUG
			UDEBUG("freqCount=%d", freqCount);
#endif

			if(_aLogFrequency->isChecked())
			{

				for(int i=0; i<img.height(); ++i)
				{
					memcpy(img.scanLine(i), _rgbLogData.at(i).data(), _rgbLogData.front().size()*sizeof(QRgb));
				}
			}
			else
			{
				for(int i=0; i<img.height(); ++i)
				{
					memcpy(img.scanLine(i), _rgbData.at(i).data(), _rgbData.front().size()*sizeof(QRgb));
				}
			}
			_imageItem->setPixmap(QPixmap::fromImage(img));

			// if scene rect has to be changed
			if(_view->sceneRect().width() > freqCount || _view->sceneRect().height() > lastFramesCount-2)
			{
				_view->scene()->removeItem(_imageItem);
				delete _view->scene();
				_view->setScene(new QGraphicsScene(this));
				_view->scene()->setBackgroundBrush(QBrush(Qt::black));
				_view->scene()->addItem(_imageItem);
			}
		}

		_view->resetTransform();
		QRectF rect = _imageItem->boundingRect();
		if(_axesSwitched)
		{
			rect = QRectF(0 ,0, rect.height(), rect.width());
		}
		int margin = 2;
		int scrollBarSize = _view->horizontalScrollBar()->height();
		QRectF viewRect = _view->rect().adjusted(margin, margin, -margin, -margin);
		if(_aScaledFreq->isChecked() && _aScaledTime->isChecked())
		{
			_view->fitInView(rect, Qt::IgnoreAspectRatio);
		}
		else if(_aScaledFreq->isChecked())
		{
			if(_axesSwitched)
			{
				if(_view->horizontalScrollBar()->isVisible())
				{
					viewRect.adjust(0, 0, 0, -scrollBarSize);
				}
				_view->scale(1, viewRect.height()/rect.height());
			}
			else
			{
				if(_view->verticalScrollBar()->isVisible())
				{
					viewRect.adjust(0, 0, -scrollBarSize, 0);
				}
				_view->scale(viewRect.width()/rect.width(), 1);
			}
		}
		else if(_aScaledTime->isChecked())
		{
			if(_axesSwitched)
			{
				if(_view->verticalScrollBar()->isVisible())
				{
					viewRect.adjust(0, 0, -scrollBarSize, 0);
				}
				_view->scale(viewRect.width()/rect.width(), 1);
			}
			else
			{
				if(_view->horizontalScrollBar()->isVisible())
				{
					viewRect.adjust(0, 0, 0, -scrollBarSize);
				}
				_view->scale(1, viewRect.height()/rect.height());
			}
		}

		if(_aZoom2x->isChecked())
		{
			if(_axesSwitched)
			{
				_view->scale(!_aScaledTime->isChecked()?2:1, !_aScaledFreq->isChecked()?2:1);
			}
			else
			{
				_view->scale(!_aScaledFreq->isChecked()?2:1, !_aScaledTime->isChecked()?2:1);
			}
		}

		if(_axesSwitched)
		{
			_view->rotate(-90);
			_view->scale(1, -1);
		}
	}
}
