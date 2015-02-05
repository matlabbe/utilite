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

#include "utilite/UPlot.h"
#include "utilite/UConversion.h"

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
				width = uStr2Float(argv[currentArg]);
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
				height = uStr2Float(argv[currentArg]);
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
							printf("Error: x axis (%d) and curve (%d) lengths are not the same!\n", (int)xAxis.size(), (int)data.size());
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
