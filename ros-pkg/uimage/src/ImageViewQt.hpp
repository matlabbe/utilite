/*
 * ImageViewQt.hpp
 *
 *  Created on: 2012-06-20
 *      Author: mathieu
 */

#ifndef IMAGEVIEWQT_HPP_
#define IMAGEVIEWQT_HPP_

#include <QtCore/QTimer>
#include <QtGui/QMouseEvent>
#include <QtGui/QApplication>
#include <QtGui/QWidget>
#include <QtGui/QPainter>
#include <QtGui/QToolTip>
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>

#include <utilite/UPlot.h>
#include <utilite/ULogger.h>

class RGBPlot: public UPlot
{
public:
	RGBPlot(int x, int y, QWidget * parent = 0) :
		x_(x),
		y_(y)
	{
		r_ = this->addCurve("R", Qt::red);
		g_ = this->addCurve("G", Qt::green);
		b_ = this->addCurve("B", Qt::blue);
	}
	~RGBPlot() {}
	void setPixel(int r, int g, int b)
	{
		r_->addValue(r);
		g_->addValue(g);
		b_->addValue(b);
	}
	int x() const {return x_;}
	int y() const {return y_;}
private:
	int x_;
	int y_;
	UPlotCurve * r_;
	UPlotCurve * g_;
	UPlotCurve * b_;
};

class ImageViewQt : public QWidget
{
	Q_OBJECT;
public:
	ImageViewQt(QWidget * parent = 0) : QWidget(parent), lastTime_(0)
	{
		this->setMouseTracking(true);
		time_.start();

		createActions();
		createMenu();
	}
	~ImageViewQt() {}

public slots:
	void setImage(const QImage & image)
	{
		lastTime_ = time_.restart();
		if(!aPause_->isChecked())
		{
			if(pixmap_.width() != image.width() || pixmap_.height() != image.height())
			{
				for(QMap<QPair<int,int>, RGBPlot*>::iterator iter = pixelMap_.begin(); iter!=pixelMap_.end();)
				{
					RGBPlot * plot = *iter;
					iter = pixelMap_.erase(iter);
					delete plot;
				}
				this->setMinimumSize(image.width(), image.height());
				this->setGeometry(this->geometry().x(), this->geometry().y(), image.width(), image.height());
			}
			else
			{
				for(QMap<QPair<int,int>, RGBPlot*>::iterator iter = pixelMap_.begin(); iter!=pixelMap_.end();++iter)
				{
					QRgb rgb = image.pixel((*iter)->x(), (*iter)->y());
					(*iter)->setPixel(qRed(rgb), qGreen(rgb), qBlue(rgb));
				}
			}
			pixmap_ = QPixmap::fromImage(image);
			if(!saveDirectory_.isEmpty())
			{
				std::string time;
				ULogger::getTime(time);
				image.save(saveDirectory_+QString("/%1.png").arg(time.c_str()));
			}
			this->update();
		}
	}

private:
	void computeScaleOffsets(float & scale, float & offsetX, float & offsetY)
	{
		scale = 1.0f;
		offsetX = 0.0f;
		offsetY = 0.0f;

		if(!pixmap_.isNull())
		{
			float w = pixmap_.width();
			float h = pixmap_.height();
			float widthRatio = float(this->rect().width()) / w;
			float heightRatio = float(this->rect().height()) / h;

			if(widthRatio < heightRatio)
			{
				scale = widthRatio;
			}
			else
			{
				scale = heightRatio;
			}

			w *= scale;
			h *= scale;

			if(w < this->rect().width())
			{
				offsetX = (this->rect().width() - w)/2.0f;
			}
			if(h < this->rect().height())
			{
				offsetY = (this->rect().height() - h)/2.0f;
			}
		}
	}
private slots:
	void removePlot(QObject * obj)
	{
		if(obj)
		{
			RGBPlot * plot = (RGBPlot*)obj;
			pixelMap_.remove(QPair<int,int>(plot->x(), plot->y()));
		}
	}

protected:
	virtual void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);
		if(!pixmap_.isNull())
		{
			painter.save();
			//Scale
			float ratio, offsetX, offsetY;
			this->computeScaleOffsets(ratio, offsetX, offsetY);
			painter.translate(offsetX, offsetY);
			painter.scale(ratio, ratio);
			painter.drawPixmap(QPoint(0,0), pixmap_);
			painter.restore();
		}
		if(aShowFrameRate_->isChecked() && lastTime_>0)
		{
			painter.setPen(QColor(Qt::green));
			painter.drawText(2, painter.font().pointSize()+2 , QString("%1 Hz").arg(1000/lastTime_));
		}
		if(aPause_->isChecked())
		{
			painter.setPen(QColor(Qt::red));
			painter.drawText(2, (painter.font().pointSize()+2)*2, "Paused");
		}
	}

	virtual void mouseMoveEvent(QMouseEvent * event)
	{
		if(!pixmap_.isNull() && aShowPixelCoordinate_->isChecked())
		{
			QPoint pos = this->mapFromGlobal(event->globalPos());

			float ratio, offsetX, offsetY;
			computeScaleOffsets(ratio, offsetX, offsetY);
			pos.rx()-=offsetX;
			pos.ry()-=offsetY;
			pos.rx()/=ratio;
			pos.ry()/=ratio;

			if(pos.x()>=0 && pos.x()<pixmap_.width() &&
			   pos.y()>=0 && pos.y()<pixmap_.height())
			{
				QToolTip::showText(event->globalPos(),
						QString("[%1,%2]")
						.arg(pos.x())
						.arg(pos.y()));
			}
		}
	}

	virtual void contextMenuEvent(QContextMenuEvent * event)
	{
		if(!pixmap_.isNull())
		{
			QPoint pos = this->mapFromGlobal(event->globalPos());

			float ratio, offsetX, offsetY;
			computeScaleOffsets(ratio, offsetX, offsetY);
			pos.rx()-=offsetX;
			pos.ry()-=offsetY;
			pos.rx()/=ratio;
			pos.ry()/=ratio;

			if(pos.x()>=0 && pos.x()<pixmap_.width() &&
			   pos.y()>=0 && pos.y()<pixmap_.height())
			{
				aPlotPixelVariation_->setText(tr("Plot pixel (%1,%2) variation").arg(pos.x()).arg(pos.y()));
				QAction * action = menu_->exec(event->globalPos());
				if(action == aPlotPixelVariation_)
				{
					if(!pixelMap_.contains(QPair<int,int>(pos.x(), pos.y())))
					{
						RGBPlot * plot = new RGBPlot(pos.x(), pos.y(), this);
						plot->setWindowTitle(tr("Pixel (%1,%2)").arg(pos.x()).arg(pos.y()));
						connect(plot, SIGNAL(destroyed(QObject *)), this, SLOT(removePlot(QObject *)));
						plot->setMaxVisibleItems(100);
						QRgb rgb = pixmap_.toImage().pixel(pos.x(), pos.y());
						plot->setPixel(qRed(rgb), qGreen(rgb), qBlue(rgb));
						plot->show();
						pixelMap_.insert(QPair<int,int>(pos.x(), pos.y()), plot);
					}
				}
				else if(action == aSaveImage_)
				{
					QString text = QFileDialog::getSaveFileName(this, tr("Save image to ..."), QString("image.png"), "*.png *.bmp *.xpm *.jpg *.pdf");
					if(!text.isEmpty())
					{
						if(!text.endsWith(".png") && !text.endsWith(".bmp") && !text.endsWith(".xpm") && !text.endsWith(".jpg") && !text.endsWith(".pdf"))
						{
							text.append(".png");//default
						}
						pixmap_.save(text);
					}
				}
				else if(action == aShowFrameRate_ || action == aPause_)
				{
					this->update();
				}
				else if(action == aSaveImages_)
				{
					if(aSaveImages_->isChecked())
					{
						QString text = QFileDialog::getExistingDirectory(this, tr("Save images to ..."), QString(""));
						if(text.isEmpty())
						{
							aSaveImages_->setChecked(false);
						}
						saveDirectory_ = text;
					}
					else
					{
						saveDirectory_ = "";
					}
				}
			}
		}
	}

	void createActions()
	{
		aShowFrameRate_ = new QAction(tr("Show frame rate"), this);
		aShowFrameRate_->setCheckable(true);
		aShowFrameRate_->setChecked(true);
		aShowPixelCoordinate_ = new QAction(tr("Show pixel coordinate"), this);
		aShowPixelCoordinate_->setCheckable(true);
		aShowPixelCoordinate_->setChecked(true);
		aPlotPixelVariation_ = new QAction(tr("Plot pixel variation"), this);
		aSaveImage_ = new QAction(tr("Save image..."), this);
		aSaveImages_ = new QAction(tr("Save images..."), this);
		aSaveImages_->setCheckable(true);
		aSaveImages_->setChecked(false);
		aPause_ = new QAction(tr("Pause"), this);
		aPause_->setCheckable(true);
		aPause_->setChecked(false);
		aPause_->setShortcut(Qt::Key_Space);
		this->connect(aPause_, SIGNAL(changed()), this, SLOT(update()));
	}

	void createMenu()
	{
		menu_ = new QMenu(this);
		menu_->addAction(aShowFrameRate_);
		menu_->addAction(aShowPixelCoordinate_);
		menu_->addAction(aPlotPixelVariation_);

		menu_->addSeparator();
		menu_->addAction(aSaveImage_);
		menu_->addAction(aSaveImages_);

		menu_->addSeparator();
		menu_->addAction(aPause_);

		this->addAction(aPause_);

	}

private:
	QPixmap pixmap_;
	QMap<QPair<int,int>, RGBPlot*> pixelMap_;
	QTime time_;
	int lastTime_;
	QString saveDirectory_;

	//menu
	QMenu * menu_;
	QAction * aShowFrameRate_;
	QAction * aShowPixelCoordinate_;
	QAction * aSaveImage_;
	QAction * aSaveImages_;
	QAction * aPause_;
	QAction * aPlotPixelVariation_;
};


#endif /* IMAGEVIEWQT_HPP_ */
