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

#include <QApplication>
#include <QString>

#include <fstream>
#include <iostream>
#include <string.h>

void showUsage()
{
	printf("Usage:\n"
			"uplot.exe value1 value2 value3... \n"
			"  Example: uplot 1.0 2.4 6.8\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}

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

	QApplication app(argc, argv);
	UPlot plot;
	plot.setGraphicsView(true);
	plot.showGrid(true);
	UPlotCurve * curve = plot.addCurve("Data");
	curve->setData(data);
	plot.show();
	app.exec();

	return 0;
}
