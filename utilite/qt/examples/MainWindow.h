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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtGui/QSlider>
#include <QtGui/QHBoxLayout>
#include <QtCore/QTimer>
#include <utilite/UPlot.h>
#include <cmath>

#define PI 3.14159265

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow() :
	t_(0),
	f1_(2), // Hz
	f2_(4), // Hz
	fs_(200) // Hz
	{
		//Plot
		UPlot * plot;
		UPlotCurve * curveSin;
		UPlotCurve * curveCos;

		// Create a plot and set some properties
		plot = new UPlot(this);
		plot->setObjectName("Figure 1");
		plot->setMaxVisibleItems(200);
		plot->showRefreshRate(true);
		plot->setXLabel("time (s)");
		plot->trackMouse(true);

		// Add curves
		curveSin = new UPlotCurve("sin", this);
		curveSin->setPen(QPen(Qt::red));
		plot->addCurve(curveSin); // ownership transferred to UPlot
		curveCos = plot->addCurve("cos", QColor(Qt::blue)); // second way to add a curve
		plot->addThreshold("zero", 0.0);
		connect(this, SIGNAL(valueUpdatedA(float, float)), curveSin, SLOT(addValue(float, float)));
		connect(this, SIGNAL(valueUpdatedB(float, float)), curveCos, SLOT(addValue(float, float)));

		//Control
		QSlider * sliderRate = new QSlider(Qt::Vertical, this);
		sliderRate->setMinimum(0); // Hz
		sliderRate->setMaximum(100); // Hz
		sliderRate->setValue(30);
		QLabel * labelRate = new QLabel(this);
		labelRate->setAlignment(Qt::AlignCenter);
		labelRate->setNum(sliderRate->value());
		connect(sliderRate, SIGNAL(valueChanged(int)), this, SLOT(setRate(int)));
		connect(sliderRate, SIGNAL(valueChanged(int)), labelRate, SLOT(setNum(int)));

		QSlider * sliderFs = new QSlider(Qt::Vertical, this);
		sliderFs->setMinimum(1); // Hz
		sliderFs->setMaximum(1000); // Hz
		sliderFs->setValue(fs_);
		QLabel * labelFs = new QLabel(this);
		labelFs->setAlignment(Qt::AlignCenter);
		labelFs->setNum(sliderFs->value());
		connect(sliderFs, SIGNAL(valueChanged(int)), this, SLOT(setFs(int)));
		connect(sliderFs, SIGNAL(valueChanged(int)), labelFs, SLOT(setNum(int)));

		QSlider * sliderF1 = new QSlider(Qt::Vertical, this);
		sliderF1->setMinimum(1); // Hz
		sliderF1->setMaximum(100); // Hz
		sliderF1->setValue(f1_);
		QLabel * labelF1 = new QLabel(this);
		labelF1->setAlignment(Qt::AlignCenter);
		labelF1->setNum(sliderF1->value());
		connect(sliderF1, SIGNAL(valueChanged(int)), this, SLOT(setF1(int)));
		connect(sliderF1, SIGNAL(valueChanged(int)), labelF1, SLOT(setNum(int)));

		QSlider * sliderF2 = new QSlider(Qt::Vertical, this);
		sliderF2->setMinimum(1); // Hz
		sliderF2->setMaximum(100); // Hz
		sliderF2->setValue(f2_);
		QLabel * labelF2 = new QLabel(this);
		labelF2->setAlignment(Qt::AlignCenter);
		labelF2->setNum(sliderF2->value());
		connect(sliderF2, SIGNAL(valueChanged(int)), this, SLOT(setF2(int)));
		connect(sliderF2, SIGNAL(valueChanged(int)), labelF2, SLOT(setNum(int)));

		// layout
		QWidget * placeHolder = new QWidget(this);
		this->setCentralWidget(placeHolder);
		QHBoxLayout * hlayout = new QHBoxLayout(placeHolder);
		hlayout->addWidget(plot, 1);

		QVBoxLayout * vlayout = new QVBoxLayout();
		vlayout->addWidget(labelRate, 0);
		vlayout->addWidget(sliderRate, 1);
		hlayout->addLayout(vlayout);

		vlayout = new QVBoxLayout();
		vlayout->addWidget(labelFs, 0);
		vlayout->addWidget(sliderFs, 1);
		hlayout->addLayout(vlayout);

		vlayout = new QVBoxLayout();
		vlayout->addWidget(labelF1, 0);
		vlayout->addWidget(sliderF1, 1);
		hlayout->addLayout(vlayout);

		vlayout = new QVBoxLayout();
		vlayout->addWidget(labelF2, 0);
		vlayout->addWidget(sliderF2, 1);
		hlayout->addLayout(vlayout);

		connect(&timer_, SIGNAL(timeout()), this, SLOT(updateCounter()));
		setRate(sliderRate->value());

		this->setMinimumWidth(720);
		this->setMinimumHeight(360);
	}
	~MainWindow() {}
public slots:
	void updateCounter() {
		emit valueUpdatedA(t_, std::sin(t_*f1_*2*PI));
		emit valueUpdatedB(t_, std::cos(t_*f2_*2*PI));
		t_ += 1/fs_;
	}
	void setRate(int rate) {
		if(rate>0)
		{
			timer_.start(1000/rate);
		}
		else
		{
			timer_.stop();
		}
	}
	void setFs(int fs) {
		fs_ = fs;
	}
	void setF1(int f1) {
		f1_ = f1;
	}
	void setF2(int f2) {
		f2_ = f2;
	}
signals:
	void valueUpdatedA(float, float);
	void valueUpdatedB(float, float);
private:
	QTimer timer_;
	float t_;
	float f1_;
	float f2_;
	float fs_;
};

#endif /* MAINWINDOW_H_ */
