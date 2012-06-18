
#include <QtGui/QApplication>
#include "MainWindow.h"

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);

	//Example 1
	UPlot plot;
	UPlotCurve * curve = plot.addCurve("My curve");
	float y[10] = {0, 1, 2, 3, -3, -2, -1, 0, 1, 2};
	curve->addValues(std::vector<float>(y, y+10));
	plot.showGrid(true);
	plot.setGraphicsView(true);
	plot.show();

	//Example 2 (advanced)
	MainWindow mainWindow;
	mainWindow.show();

	app.connect( &app, SIGNAL( lastWindowClosed() ),
				 &app, SLOT( quit() ) );
	return app.exec();
}
