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

#include "utilite/UPlot.h"

#include <QtGui/QApplication>
#include <QtCore/QString>
#include <QtCore/QFile>
#include <QtCore/QTimer>

#include <fstream>
#include <iostream>
#include <string.h>
#include <cstdlib>

void showUsage()
{
	printf("Usage:\n"
			"uplot.exe [options] value1 value2 value3... \n"
			"uplot.exe data.txt \n"
			"  Example: uplot 1.0 2.4 6.8\n"
			"Options:\n"
			"  -t \"My title\"        Plot title\n"
			"  -x \"My axis name\"    X axis label\n"
			"  -y \"My axis name\"    Y axis label\n"
			"  -w #                   Window width\n"
			"  -h #                   Window height\n"
			"File format example (for multiple curves on the same plot):\n"
			"   -time 1 2 3 4         The \"-\" means x axis, and \"time\" is\n"
			"                         the axis name\n"
			"   curve_1 5 23 12 12    Curve with name \"curve 1\" (the underscores\n"
			"                         are replaced by spaces)\n"
			"   curve_2 2 8 12 56    Another curve with name \"curve 2\"\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}

	QApplication app(argc, argv);
	UPlot plot;
	plot.setWindowIcon(QIcon(":Plot48.png"));

	//parse options
	QString title;
	QString xLabel;
	QString yLabel;
	int width=0;
	int height=0;
	int currentArg = 1;
	for(; currentArg<argc; ++currentArg)
	{
		if(strcmp(argv[currentArg], "-t") == 0)
		{
			++currentArg;
			if(currentArg < argc)
			{
				title = argv[currentArg];
			}
			else
			{
				showUsage();
			}
			continue;
		}
		else if(strcmp(argv[currentArg], "-w") == 0)
		{
			++currentArg;
			if(currentArg < argc)
			{
				width = std::atof(argv[currentArg]);
				if(width < 0)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		else if(strcmp(argv[currentArg], "-h") == 0)
		{
			++currentArg;
			if(currentArg < argc)
			{
				height = std::atof(argv[currentArg]);
				if(height < 0)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[currentArg], "-x") == 0)
		{
			++currentArg;
			if(currentArg < argc)
			{
				xLabel = argv[currentArg];
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[currentArg], "-y") == 0)
		{
			++currentArg;
			if(currentArg < argc)
			{
				yLabel = argv[currentArg];
			}
			else
			{
				showUsage();
			}
			continue;
		}
		else
		{
			break;
		}
	}
	//parse options end
	plot.setTitle(title);
	plot.setXLabel(xLabel);
	plot.setYLabel(yLabel);
	if(width)
	{
		plot.setGeometry(plot.geometry().x(), plot.geometry().y(), width, plot.geometry().height());
	}
	if(height)
	{
		plot.setGeometry(plot.geometry().x(), plot.geometry().y(), plot.geometry().width(), height);
	}

	bool number = false;
	QString(argv[currentArg]).toFloat(&number);
	if(number)
	{
		plot.showGrid(true);
		plot.setGraphicsView(true);
		std::vector<float> data;
		for(int i=currentArg; i<argc ;++i)
		{
			bool ok = false;
			float value = QString(argv[i]).toFloat(&ok);
			if(!ok)
			{
				printf("\nError parsing value %s to float", argv[i]);
				return -1;
			}
			data.push_back(value);
		}
		UPlotCurve * curve = plot.addCurve("Data");
		curve->setData(data);
	}
	else
	{
		///file...
		QFile file(argv[currentArg]);
		if(file.open(QIODevice::ReadOnly))
		{
			//parse the file
			QString str(file.readAll());
			QStringList curves = str.split('\n');
			std::vector<float> xAxis;
			QString xAxisName;
			int curveId = 0;
			for(int j=0; j<curves.size(); ++j)
			{
				QStringList values = curves[j].simplified().split(' ');
				if(values.size() && values[0].trimmed().size())
				{
					std::vector<float> data;
					QString dataName;
					bool isXAxis = false;
					for(int i=0; i<values.size() ;++i)
					{
						bool ok = false;
						float value = values[i].toFloat(&ok);
						if(ok)
						{
							if(!isXAxis)
							{
								if(i==0)
								{
									++curveId;
								}
								data.push_back(value);
							}
							else
							{
								xAxis.push_back(value);
							}
						}
						else if(i==0)
						{
							if(values[i].size() && values[i].at(0) == '-')
							{
								isXAxis = true;
								xAxis.clear();
								xAxisName.clear();
								if(values[i].size() > 1)
								{
									values[i].remove(0,1);
									values[i].replace('_', ' ');
									xAxisName = values[i];
								}
							}
							else if(values[i].size())
							{
								++curveId;
								values[i].replace('_', ' ');
								dataName = values[i];
							}
						}
						else
						{
							printf("Error parsing value %s\n", values[i].toStdString().c_str());
							return -1;
						}

					}
					if(!isXAxis)
					{
						UPlotCurve * curve = plot.addCurve(QString("%1").arg(dataName.size()?dataName:QString("Data%1").arg(curveId)));
						printf("Adding curve \"%s\"\n", plot.curveNames().last().toStdString().c_str());
						if(xAxis.size() && xAxis.size() == data.size())
						{
							curve->setData(xAxis, data);
						}
						else if(xAxis.size() == 0)
						{
							curve->setData(data);
						}
						else
						{
							printf("Error: x axis and curve (%d) length is not the same!\n", j);
							return -1;
						}
					}
					else
					{
						printf("X axis set: \"%s\"\n", xAxisName.toStdString().c_str());
					}
				}
			}
			if(xLabel.isEmpty())
			{
				plot.setXLabel(xAxisName);
			}
			if(title.isEmpty())
			{
				plot.setTitle(file.fileName());
			}
		}
		else
		{
			printf("\n Cannot open file %s", argv[1]);
		}
	}

	plot.show();
	QTimer::singleShot(10, &plot, SLOT(updateAxis()));

	app.exec();

	return 0;
}
