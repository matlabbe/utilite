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

#ifndef UPLOT_H_
#define UPLOT_H_

#include "utilite/UtiLiteExp.h" // DLL export/import defines

#include <QtGui/QFrame>
#include <QtCore/QList>
#include <QtCore/QMap>
#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsEllipseItem>
#include <QtCore/QMutex>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtCore/QTime>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsItem;
class QFormLayout;

class UTILITE_EXP UPlotItem : public QGraphicsEllipseItem
{
public:
	UPlotItem(qreal dataX, qreal dataY, qreal width=2);
	UPlotItem(const QPointF & data, qreal width=2);
	virtual ~UPlotItem();

public:
	void setNextItem(UPlotItem * nextItem);
	void setPreviousItem(UPlotItem * previousItem);
	void setData(const QPointF & data);

	UPlotItem * nextItem() const {return _nextItem;}
	UPlotItem * previousItem() const {return _previousItem;};
	const QPointF & data() const {return _data;}

protected:
	virtual void hoverEnterEvent(QGraphicsSceneHoverEvent * event);
	virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);
	virtual void focusInEvent(QFocusEvent * event);
	virtual void focusOutEvent(QFocusEvent * event);
	virtual void keyReleaseEvent(QKeyEvent * keyEvent);

	virtual void showDescription(bool shown);

private:
	QPointF _data;
	QGraphicsTextItem * _text;
	UPlotItem * _previousItem;
	UPlotItem * _nextItem;
};

class UPlot;

class UTILITE_EXP UPlotCurve : public QObject
{
	Q_OBJECT

public:
	UPlotCurve(const QString & name, QObject * parent = 0);
	UPlotCurve(const QString & name, const QVector<UPlotItem *> data, QObject * parent = 0);
	UPlotCurve(const QString & name, const QVector<float> & x, const QVector<float> & y, QObject * parent = 0);
	virtual ~UPlotCurve();

	const QPen & pen() const {return _pen;}
	const QBrush & brush() const {return _brush;}
	void setPen(const QPen & pen);
	void setBrush(const QBrush & brush);

	QString name() const {return _name;}
	int itemsSize() const;
	QPointF getItemData(int index);
	bool isVisible() const {return _visible;}
	void setData(QVector<UPlotItem*> & data); // take the ownership
	void setData(const QVector<float> & x, const QVector<float> & y);
	void setData(const std::vector<float> & x, const std::vector<float> & y);
	void setData(const std::vector<float> & y);
	void getData(QVector<float> & x, QVector<float> & y) const; // only call in Qt MainThread
	void draw(QPainter * painter);

public slots:
	virtual void clear();
    void setVisible(bool visible);
    void setXIncrement(float increment);
    void setXStart(float val);
	void addValue(UPlotItem * data); // take the ownership
	void addValue(float y);
	void addValue(float x, float y);
	void addValue(const QString & y);
	void addValues(QVector<UPlotItem *> & data); // take the ownership
	void addValues(const QVector<float> & xs, const QVector<float> & ys);
	void addValues(const QVector<float> & ys);
	void addValues(const QVector<int> & ys); // for convenience
	void addValues(const std::vector<float> & ys); // for convenience
	void addValues(const std::vector<int> & ys); // for convenience

signals:
	void dataChanged(const UPlotCurve *);

protected:
	friend class UPlot;
	void attach(UPlot * plot);
	void detach(UPlot * plot);
	void updateMinMax();
	const QVector<float> & getMinMax() const {return _minMax;}
	int removeItem(int index);
	void _addValue(UPlotItem * data);;
	virtual bool isMinMaxValid() const {return _minMax.size();}
	virtual void update(float scaleX, float scaleY, float offsetX, float offsetY, float xDir, float yDir, bool allDataKept);
	QList<QGraphicsItem *> _items;
	UPlot * _plot;

private:
	void removeItem(UPlotItem * item);

private:
	QString _name;
	QPen _pen;
	QBrush _brush;
	float _xIncrement;
	float _xStart;
	bool _visible;
	bool _valuesShown;
	QVector<float> _minMax; // minX, maxX, minY, maxY
};


class UTILITE_EXP UPlotCurveThreshold : public UPlotCurve
{
	Q_OBJECT

public:
	UPlotCurveThreshold(const QString & name, float thesholdValue, Qt::Orientation orientation = Qt::Horizontal, QObject * parent = 0);
	virtual ~UPlotCurveThreshold();

public slots:
	void setThreshold(float threshold);
	void setOrientation(Qt::Orientation orientation);

protected:
	friend class UPlot;
	virtual void update(float scaleX, float scaleY, float offsetX, float offsetY, float xDir, float yDir, bool allDataKept);
	virtual bool isMinMaxValid() const {return false;}

private:
	Qt::Orientation _orientation;
};


class UTILITE_EXP UPlotAxis : public QWidget
{
public:
	UPlotAxis(Qt::Orientation orientation = Qt::Horizontal, float min=0, float max=1, QWidget * parent = 0);
	virtual ~UPlotAxis();

public:
	void setAxis(float & min, float & max);
	int border() const {return _border;}
	int step() const {return _step;}
	int count() const {return _count;}
	void setReversed(bool reversed); // Vertical :bottom->up, horizontal :right->left

protected:
	virtual void paintEvent(QPaintEvent * event);

private:
	Qt::Orientation _orientation;
	float _min;
	float _max;
	int _count;
	int _step;
	bool _reversed;
	int _gradMaxDigits;
	int _border;
};


class UTILITE_EXP UPlotLegendItem : public QPushButton
{
	Q_OBJECT

public:
	UPlotLegendItem(const UPlotCurve * curve, QWidget * parent = 0);
	virtual ~UPlotLegendItem();
	const UPlotCurve * curve() const {return _curve;}

signals:
	void legendItemRemoved(const UPlotCurve *);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private:
	const UPlotCurve * _curve;
	QMenu * _menu;
	QAction * _aChangeText;
	QAction * _aResetText;
	QAction * _aRemoveCurve;
	QAction * _aCopyToClipboard;
};


class UTILITE_EXP UPlotLegend : public QWidget
{
	Q_OBJECT

public:
	UPlotLegend(QWidget * parent = 0);
	virtual ~UPlotLegend();

	void setFlat(bool on);
	bool isFlat() const {return _flat;}
	void addItem(const UPlotCurve * curve);
	QPixmap createSymbol(const QPen & pen, const QBrush & brush);
	bool remove(const UPlotCurve * curve);

public slots:
	void removeLegendItem(const UPlotCurve * curve);

signals:
	void legendItemRemoved(const UPlotCurve * curve);
	void legendItemToggled(const UPlotCurve * curve, bool toggled);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);

private slots:
	void redirectToggled(bool);

private:
	bool _flat;
	QMenu * _menu;
	QAction * _aUseFlatButtons;
};


class UTILITE_EXP UPlotOrientableLabel : public QLabel
{
	Q_OBJECT

public:
	UPlotOrientableLabel(const QString & text, Qt::Orientation orientation = Qt::Horizontal, QWidget * parent = 0);
	virtual ~UPlotOrientableLabel();
	Qt::Orientation orientation() const {return _orientation;}
	void setOrientation(Qt::Orientation orientation);
	QSize sizeHint() const;
	QSize minimumSizeHint() const;
protected:
    virtual void paintEvent(QPaintEvent* event);
private:
    Qt::Orientation _orientation;
};

/*
 * TODO It could be cool to right-click on a dot in the
 * plot to create a new plot to monitor the dot value changes.
 */
class UTILITE_EXP UPlot : public QWidget
{
	Q_OBJECT

public:
	UPlot(QWidget * parent = 0);
	virtual ~UPlot();

	UPlotCurve * addCurve(const QString & curveName);
	bool addCurve(UPlotCurve * curves);
	QStringList curveNames();
	bool contains(const QString & curveName);
	void removeCurves();
	UPlotCurveThreshold * addThreshold(const QString & name, float value, Qt::Orientation orientation = Qt::Horizontal);
	QString title() const {return this->objectName();}
	QPen getRandomPenColored();
	void showLegend(bool shown);
	void showGrid(bool shown);
	void showRefreshRate(bool shown);
	void keepAllData(bool kept);
	void showXAxis(bool shown) {_horizontalAxis->setVisible(shown);}
	void showYAxis(bool shown) {_verticalAxis->setVisible(shown);}
	void setVariableXAxis() {_fixedAxis[0] = false;}
	void setVariableYAxis() {_fixedAxis[1] = false;}
	void setFixedXAxis(float x1, float x2);
	void setFixedYAxis(float y1, float y2);
	void setMaxVisibleItems(int maxVisibleItems);
	void setTitle(const QString & text);
	void setXLabel(const QString & text);
	void setYLabel(const QString & text, Qt::Orientation orientation = Qt::Vertical);
	void setWorkingDirectory(const QString & workingDirectory);
	void setGraphicsView(bool on);
	QRectF sceneRect() const;

public slots:
	void removeCurve(const UPlotCurve * curve);
	void showCurve(const UPlotCurve * curve, bool shown);
	void updateAxis(); //reset axis and recompute it with all curves minMax
	void clearData();

private slots:
	void captureScreen();
	void updateAxis(const UPlotCurve * curve);

protected:
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void paintEvent(QPaintEvent * event);
	virtual void resizeEvent(QResizeEvent * event);

private:
	friend class UPlotCurve;
	void addItem(QGraphicsItem * item);

private:
	void replot(QPainter * painter);
	bool updateAxis(float x, float y);
	bool updateAxis(float x1, float x2, float y1, float y2);
	void setupUi();
	void createActions();
	void createMenus();
	void selectScreenCaptureFormat();

private:
	UPlotLegend * _legend;
	QGraphicsView * _view;
	QGraphicsItem * _sceneRoot;
	QWidget * _graphicsViewHolder;
	float _axisMaximums[4]; // {x1->x2, y1->y2}
	bool _axisMaximumsSet[4]; // {x1->x2, y1->y2}
	bool _fixedAxis[2];
	UPlotAxis * _verticalAxis;
	UPlotAxis * _horizontalAxis;
	int _penStyleCount;
	int _maxVisibleItems;
	QList<QGraphicsLineItem *> hGridLines;
	QList<QGraphicsLineItem *> vGridLines;
	QList<UPlotCurve*> _curves;
	QLabel * _title;
	QLabel * _xLabel;
	UPlotOrientableLabel * _yLabel;
	QLabel * _refreshRate;
	QString _workingDirectory;
	QTime _refreshIntervalTime;
	int _lowestRefreshRate;
	QTime _refreshStartTime;
	QString _autoScreenCaptureFormat;

	QMenu * _menu;
	QAction * _aShowLegend;
	QAction * _aShowGrid;
	QAction * _aKeepAllData;
	QAction * _aLimit0;
	QAction * _aLimit10;
	QAction * _aLimit50;
	QAction * _aLimit100;
	QAction * _aLimit500;
	QAction * _aLimit1000;
	QAction * _aLimitCustom;
	QAction * _aAddVerticalLine;
	QAction * _aAddHorizontalLine;
	QAction * _aChangeTitle;
	QAction * _aChangeXLabel;
	QAction * _aChangeYLabel;
	QAction * _aYLabelVertical;
	QAction * _aShowRefreshRate;
	QAction * _aSaveFigure;
	QAction * _aAutoScreenCapture;
	QAction * _aClearData;
	QAction * _aGraphicsView;
};

#endif /* UPLOT_H_ */
