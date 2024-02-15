#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "QPaintEvent"
#include "ui_widget.h"
#include<opencv2/opencv.hpp>
#include<QTimer>
#include<pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkSmartPointer.h"
#include "QtDataVisualization"
#include "QDateTime.h"
#include <QTimerEvent>


using namespace cv;
using namespace pcl;
using namespace QtDataVisualization;

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
     void InitGraph3D();

private slots:
    void importFrame();
    void on_displayButton_clicked();
    void on_stopButton_clicked();
    void on_viewerButton_clicked();
    void on_cycleButton_clicked();
    void show_anglepao(double angle_downandup,double angle_leftandright, double angle2_downandup,double angle2_leftandright,
                       int DU1,int LR1,int DU2,int LR2);

private:
    virtual void  timerEvent( QTimerEvent * event );


private:
    Ui::Widget *ui=new Ui::Widget();
    VideoCapture capture;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindowInteractor> iren;
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style;

    QTimer *timer=new QTimer();
    Mat frame;
    bool isCamera=0;
    bool m_bClick=false;
    QPoint m_Point;
    int m_Zoom=100;

    QDateTime *current_date_time=new QDateTime();



};
#endif // WIDGET_H
