#include "widget.h"

#include <QApplication>
#include <vtkOutputWindow.h>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    vtkOutputWindow::SetGlobalWarningDisplay(0);//
    Widget w;
    w.setWindowTitle(QString("料堆智能融合视觉识别系统"));
    w.show();
    return a.exec();
}
