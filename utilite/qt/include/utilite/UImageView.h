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

#ifndef IMAGEVIEW_H_
#define IMAGEVIEW_H_

#include "utilite/UtiLiteQtExp.h" // DLL export/import defines


#include <QtGui/QWidget>
#include <QtGui/QPainter>
#include <QtGui/QFileDialog>
#include <QtGui/QMouseEvent>
#include <QtCore/QDir>
#include <QtCore/QDateTime>

/**
 * UImageView is a QWidget to show an image, which
 * scales automatically when resizing the widget. Save the
 * image on right-click.
 *
 * @see setImage()
 *
 */
class UTILITEQT_EXP UImageView : public QWidget
{
	Q_OBJECT;
public:
	UImageView(QWidget * parent = 0) :
		QWidget(parent)
	{}
	~UImageView() {}
	void setBackgroundBrush(const QBrush & brush) {brush_ = brush;}

public slots:
	void setImage(const QImage & image)
	{
		if(pixmap_.width() != image.width() || pixmap_.height() != image.height())
		{
			this->setMinimumSize(image.width(), image.height());
			this->setGeometry(this->geometry().x(), this->geometry().y(), image.width(), image.height());
		}
		pixmap_ = QPixmap::fromImage(image);
		this->update();
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

protected:
	virtual void paintEvent(QPaintEvent *event)
	{
		QPainter painter(this);

		//Draw background
		painter.save();
		painter.setBrush(brush_);
		painter.drawRect(this->rect());
		painter.restore();

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
	}

	void mousePressEvent(QMouseEvent * e)
	{
		if(e->button() == Qt::RightButton)
		{
			QString name = (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + ".png");
			pixmap_.save(name);
			printf("Saved screenshot \"%s\"\n", name.toStdString().c_str());
		}
	}

private:
	QPixmap pixmap_;
	QBrush brush_;
};


#endif /* IMAGEVIEW_H_ */
