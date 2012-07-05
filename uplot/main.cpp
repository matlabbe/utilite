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

#include <fstream>
#include <iostream>
#include <string.h>

void showUsage()
{
	printf("Usage:\n"
			"uplot.exe value1 value2 value3... \n"
			"uplot.exe data.txt \n"
			"  Example: uplot 1.0 2.4 6.8\n");
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

	bool number = false;
	QString(argv[1]).toFloat(&number);
	if(number)
	{
		plot.showGrid(true);
		plot.setGraphicsView(true);
		std::vector<float> data;
		for(int i=1; i<argc ;++i)
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
		QFile file(argv[1]);
		if(file.open(QIODevice::ReadOnly))
		{
			//parse the file
			QString str(file.readAll());
			QStringList curves = str.split('\n');
			for(int j=0; j<curves.size(); ++j)
			{
				QStringList values = curves[j].simplified().split(' ');
				if(values.size() && values[0].trimmed().size())
				{
					printf("values=%d\n", values.size());
					std::vector<float> data;
					for(int i=0; i<values.size() ;++i)
					{
						bool ok = false;
						float value = values[i].toFloat(&ok);
						if(!ok)
						{
							printf("\nError parsing value \"%s\" to float", argv[i]);
							return -1;
						}
						data.push_back(value);
					}
					UPlotCurve * curve = plot.addCurve(QString("Data%1").arg(j));
					curve->setData(data);
				}
			}
		}
		else
		{
			printf("\n Cannot open file %s", argv[1]);
		}
	}

	plot.show();
	app.exec();

	return 0;
}
