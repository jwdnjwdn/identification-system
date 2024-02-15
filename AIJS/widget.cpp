#include "widget.h"
#include "ui_widget.h"
#include <QImage>
#include "QFileDialog"
#include "pcl/io/io.h"
#include "pcl/io/ply_io.h"
#include "pcl/filters/passthrough.h"
#include "pcl/point_types.h"
#include "pcl/features/normal_3d.h"
#include "pcl/common/transforms.h"
#include "pcl/common/common.h"
#include "pcl/common/common_headers.h"
#include "pcl/kdtree/kdtree_flann.h" //kdtree类定义头文件
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/visualization/cloud_viewer.h"
#include <stdio.h>
#include "vtkRenderWindow.h"
#include "vtkOutputWindow.h"
#include <vtkAutoInit.h>
#include "QPainter"
#include <QtDataVisualization>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkElevationFilter.h>
#include <vtkNamedColors.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkMassProperties.h>
#include <vtkSTLReader.h>
#include <vtkCubeAxesActor.h>
#include <vtkAppendPolyData.h>
#include <vtkCamera.h>
#include <iostream>
#include <QDateTime>
#include <vector>

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

using namespace std;
vector<QString> logMessages;
vector<QString> logMes;

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    timer=new QTimer(this);
    //this->setWindowFlags(this->windowFlags() | Qt::FramelessWindowHint);
    //this->setWindowFlags(Qt::CustomizeWindowHint);
    connect(timer,SIGNAL(timeout()),this,SLOT(importFrame()));

    // 启动定时器
    timer->start();

}

Widget::~Widget()
{
    delete ui;
}


void processcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp1_cloud(new PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(incloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 4.5);
    pass.filter(*temp_cloud);
    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(temp_cloud);
    pass2.setFilterFieldName("y");
    pass2.setFilterLimits(-5.0, 1.0);
    pass2.filter(*temp1_cloud);

    Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();//
    double angle_z = -M_PI * 45 / 180;//
    rotation_z(0, 0) = cos(angle_z);
    rotation_z(0, 1) = -sin(angle_z);
    rotation_z(1, 0) = sin(angle_z);
    rotation_z(1, 1) = cos(angle_z);

    Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();//
    double angle_y = -M_PI / 18;//rot90°
    rotation_y(0, 0) = cos(angle_y);
    rotation_y(0, 2) = sin(angle_y);
    rotation_y(2, 0) = -sin(angle_y);
    rotation_y(2, 2) = cos(angle_y);

    Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity();//
    double angle_x = -M_PI * 5 / 180;//
    rotation_x(1, 1) = cos(angle_x);
    rotation_x(1, 2) = -sin(angle_x);
    rotation_x(2, 1) = sin(angle_x);
    rotation_x(2, 2) = cos(angle_x);

    Eigen::Matrix4f rotation_xz = Eigen::Matrix4f::Identity();//
    rotation_xz = rotation_x * rotation_z * rotation_y;
    pcl::transformPointCloud(*temp1_cloud, *temp1_cloud, rotation_xz);


    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp2_cloud(new PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new PointCloud<pcl::PointXYZ>);
   // pcl::PassThrough<pcl::PointXYZ> pass3;
//    pass3.setInputCloud(temp1_cloud);
//    pass3.setFilterFieldName("x");
//    //    pass3.setFilterLimits (-2.0, 1.8);
//    pass3.setFilterLimits(-1.7, 1.4);
//    pass3.filter(*temp2_cloud);

//    pcl::PassThrough<pcl::PointXYZ> pass4;
//    pass4.setInputCloud(temp2_cloud);
//    pass4.setFilterFieldName("y");
//    pass4.setFilterLimits(-5.0, -2.0);
//    pass4.filter(*outcloud);

    for (int i = 0; i < temp1_cloud->size(); i++) {
        if (temp1_cloud->points[i].x > -1.7 && temp1_cloud->points[i].x<1.40 &&
            temp1_cloud->points[i].y>-3.8 && temp1_cloud->points[i].y < -2.3)
        {
            final_cloud->push_back(temp1_cloud->points[i]);
        }

    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud(final_cloud);
    sor2.setMeanK(50);
    sor2.setStddevMulThresh(5.0);
    sor2.filter(*outcloud);
}

void drawPlane(vtkAppendPolyData* appendFilter, PointXYZ p1, PointXYZ p2, PointXYZ p3, PointXYZ p4) {
    vtkPolyData* planePld = vtkPolyData::New();
    vtkPoints* planePoints = vtkPoints::New();
    vtkCellArray* planeCells = vtkCellArray::New();

    planePoints->SetNumberOfPoints(4);
    planePoints->InsertPoint(0, p1.x, p1.y, p1.z);
    planePoints->InsertPoint(1, p2.x, p2.y, p2.z);
    planePoints->InsertPoint(2, p3.x, p3.y, p3.z);
    planePoints->InsertPoint(3, p4.x, p4.y, p4.z);
    planePld->SetPoints(planePoints);
    vtkIdType cellId1[3] = { 1, 2, 3 };
    vtkIdType cellId2[3] = { 1, 0, 3 };
    //Create a cell by specifying the number of points and an array of pointid's.  Return the cell id of the cell.
    planeCells->InsertNextCell(3, cellId1);
    planeCells->InsertNextCell(3, cellId2);
    planePld->SetPolys(planeCells);

    appendFilter->AddInputData(planePld);
    planePld->Delete();
    planePoints->Delete();
    planeCells->Delete();
}

QString log_message(int CODE1,int CODE2,QString str){

    if(CODE1==0&&CODE2==0){
        QDateTime dateTime= QDateTime::currentDateTime();
        QString date = dateTime.toString("yyyy-MM-dd hh:mm:ss");
        date.append(str);
        logMessages.push_back(date);
    }

    if(CODE1==1){
        QDateTime dateTime= QDateTime::currentDateTime();
        QString date = dateTime.toString("yyyy-MM-dd hh:mm:ss");
        date.append('-');
        logMes.push_back(date);
        if(CODE2==1){
            logMes.push_back(date+str);
            QString Logs;
            for (const QString& log : logMes) {
                Logs += log;
            }
            logMessages.push_back(Logs);
        }
    }

    QString allLogs;
    for (const QString& log : logMessages) {
        allLogs += log + "\n";
    }
    return allLogs;
}

void Widget::show_anglepao(double angle_downandup,double angle_leftandright, double angle2_downandup,double angle2_leftandright,
                   int DU1,int LR1,int DU2,int LR2){
    QString du1=" ";
    if(DU1==1){
        du1="down : ";
    }else if(DU1==2){
        du1="up : ";
    }else{
        du1=" ";
    }

    QString lr1=" ";
    if(LR1==1){
        lr1="left : ";
    }else if(LR1==2){
        lr1="right : ";
    }else{
        lr1=" ";
    }

    QString str_pao1_ud = QString::number(angle_downandup);
    du1.append(str_pao1_ud);
    ui->Pao1_up_label->setText(du1);
    ui->Pao1_up_label->show();


    QString str_pao1_lr = QString::number(angle_leftandright);  //
    lr1.append(str_pao1_lr);
    ui->Pao1_down_label->setText(lr1);
    ui->Pao1_down_label->show();



    QString du2=" ";
    if(DU2==1){
        du2="down : ";
    }else if(DU2==2){
        du2="up : ";
    }else{
        du2=" ";
    }

    QString lr2=" ";
    if(LR2==1){
        lr2="left : ";
    }else if(LR2==2){
        lr2="right : ";
    }else{
        lr2=" ";
    }
    QString str_pao2_ud = QString::number(angle2_downandup);  //
    du2.append(str_pao2_ud);
    ui->Pao2_up_label->setText(du2);
    ui->Pao2_up_label->show();  //

    QString str_pao2_lr = QString::number(angle2_leftandright);
    lr2.append(str_pao2_lr);
    ui->Pao2_down_label->setText(lr2);
    ui->Pao2_down_label->show();  //
}

//void Widget::on_stopButton_clicked()
//{
//    timer->stop();
//    capture.release();
//}

void Widget::on_displayButton_clicked()
{

    if(isCamera){
        capture.open(0);
        QString camera_s=" Open camera failure...";
        ui->label_9->setText(log_message(0,0,camera_s));
        ui->label_9->show();
    }else{

        capture.open("G:\\waibao\\leishen_16\\4.mp4");
        QString camera_s=" Open camera successful...";
        ui->label_9->setText(log_message(0,0,camera_s));
        ui->label_9->show();

    }
    timer->start(1);

}


void Widget::importFrame(){
    ui->displayButton->setStyleSheet("background-color: rgb(0,255,0);color:rgb(255,255,255);");
    capture>>frame;
    if(frame.empty()){
        ui->displaylabel->close();
    }
    cvtColor(frame,frame,CV_BGR2RGB);
    int width=ui->displaylabel->width();
    int height=ui->displaylabel->height();
    QImage srcQimage=QImage((uchar*)(frame.data),frame.cols,frame.rows,QImage::Format_RGB888);

    QPixmap pixmap=QPixmap::fromImage(srcQimage);
    QPixmap fitpixmap=pixmap.scaled(width,height,Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
    ui->displaylabel->setPixmap(fitpixmap);
    ui->displaylabel->resize(fitpixmap.size());
    ui->displaylabel->show();
}


void Widget::on_viewerButton_clicked(){

    QString keyconnect=" Connect the Lidar...";
    ui->label_9->setText(log_message(0,0,keyconnect));
    ui->label_9->show();

    vtkSmartPointer<vtkPoints> points =vtkSmartPointer<vtkPoints>::New();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("G:\\waibao\\leishen_16\\2023-10-04-00-28.pcd",*cloud);
    processcloud(cloud, out_cloud);

    QString deal=" start the pointcloud...";
    ui->label_9->setText(log_message(0,0,deal));
    ui->label_9->show();

    std::string fire_filename = "D:\\vs2019\\qt_pro\\AIJS\\xiaofangpao.STL";
    vtkSmartPointer<vtkSTLReader> read_fire = vtkSmartPointer<vtkSTLReader>::New();
    read_fire->SetFileName(fire_filename.c_str());
    read_fire->Update();

    vtkSmartPointer<vtkPolyData> poly_fire = vtkSmartPointer<vtkPolyData>::New();
    poly_fire = read_fire->GetOutput();

    vtkSmartPointer<vtkPolyData> poly_fire2 = vtkSmartPointer<vtkPolyData>::New();
    poly_fire2 = read_fire->GetOutput();

    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Scale(0.001,0.001,0.001);
    transform->Translate(1400,-2700,2000);
    transform->RotateX(80);
    transform->RotateY(180);
    transform->Update();

    vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
    transform2->Scale(0.001, 0.001, 0.001);
    transform2->Translate(1400, -2700, 0);
    transform2->RotateX(80);
    transform2->RotateY(180);
    transform2->Update();


    vtkSmartPointer<vtkTransformPolyDataFilter> trans_Filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    trans_Filter->SetInputData(poly_fire);
    trans_Filter->SetTransform(transform);
    trans_Filter->Update();

    vtkSmartPointer<vtkTransformPolyDataFilter> trans_Filter2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    trans_Filter2->SetInputData(poly_fire2);
    trans_Filter2->SetTransform(transform2);
    trans_Filter2->Update();


    for (unsigned int i = 0; i < out_cloud->size(); i++)
    {
         points->InsertNextPoint(out_cloud->points[i].x, out_cloud->points[i].y, out_cloud->points[i].z);
    }
    int record_miny = -1, record_maxy = -1;
    double maxy = VTK_DOUBLE_MIN, miny = VTK_DOUBLE_MAX,
           maxx = VTK_DOUBLE_MIN, minx = VTK_DOUBLE_MAX,
           maxz = VTK_DOUBLE_MIN, minz = VTK_DOUBLE_MAX;
    for (unsigned int i = 0; i < out_cloud->size(); i++)
    {
        if (out_cloud->points[i].x > maxx) {
            maxx = out_cloud->points[i].x;
        }
        if (out_cloud->points[i].x < minx) {
            minx = out_cloud->points[i].x;
        }
        if (out_cloud->points[i].y > maxy) {
            maxy = out_cloud->points[i].y;
            record_maxy = i;
        }
        if (out_cloud->points[i].y < miny) {
            miny = out_cloud->points[i].y;
            record_miny = i;
        }
        if (out_cloud->points[i].z > maxz) {
            maxz = out_cloud->points[i].z;
        }
        if (out_cloud->points[i].z < minz) {
            minz = out_cloud->points[i].z;
        }

    }
    double interval_x = (maxx - minx) / 5;
    double interval_z = (maxz - minz) / 5;
    int grid_x = int((out_cloud->points[record_maxy].x - minx) / interval_x)+1;
    int grid_z = int((out_cloud->points[record_maxy].z - minz) / interval_z)+1;


    //height point label
    QString str_highpoint_x = QString::number((round(cloud->points[record_maxy].x * 100) / 100));  //
    ui->high_X_label->setText(str_highpoint_x);
    ui->high_X_label->show();  // show QLabel

    QString str_highpoint_y = QString::number((round(cloud->points[record_maxy].y * 100) / 100));  //
    ui->high_Y_label->setText(str_highpoint_y);
    ui->high_Y_label->show();  // show QLabel

    QString str_highpoint_z = QString::number((round(cloud->points[record_maxy].z * 100) / 100));  //
    ui->high_Z_label->setText(str_highpoint_z);
    ui->high_Z_label->show();  // show QLabel

    //grid location
    QString str_mesh_x = QString::number(grid_x);  //
    ui->mesh_X_label->setText(str_mesh_x);
    ui->mesh_X_label->show();  // show QLabel

    QString str_mesh_z = QString::number(grid_z);  //
    ui->mesh_Z_label->setText(str_mesh_z);
    ui->mesh_Z_label->show();  // show QLabel

    //max_volume location
    QString str_max_x = QString::number(grid_x);  //
    ui->max_X_label->setText(str_max_x);
    ui->max_X_label->show();  // show QLabel

    QString str_max_z = QString::number(grid_z);  //
    ui->max_Z_label->setText(str_max_z);
    ui->max_Z_label->show();  // show QLabel

    //show grid down-border
    double line_x=minx+grid_x*interval_x;
    QString str_download_x = QString::number(round(line_x*100)/100);  //
    ui->down_X_label->setText(str_download_x);
    ui->down_X_label->show();  //

    double line_z=minz+grid_z*interval_z;
    QString str_download_z = QString::number(round(line_z*100)/100);  //
    ui->down_Z_label->setText(str_download_z);
    ui->down_Z_label->show();

    show_anglepao(20,45,20,30,1,2,1,1);

    //生成围栏a
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(1.62,-4.0,2.2)));
    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(1.62, -2.8, 2.2)));
    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(1.62, -2.8, -0.5)));
    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(1.62, -4.0, -0.5)));

    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(-1.7, -4.0, 2.2)));
    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(-1.7, -3.0, 2.2)));
    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(-1.7, -3.0, -0.5)));
    plane_cloud->push_back(pcl::PointXYZ(pcl::PointXYZ(-1.7, -4.0, -0.5)));


    vtkPolyData* polyDataGlobe = vtkPolyData::New();
    vtkAppendPolyData* appendFilter = vtkAppendPolyData::New();

    drawPlane(appendFilter, out_cloud->points[0], out_cloud->points[1], out_cloud->points[2], out_cloud->points[3]);
    appendFilter->Update();

    //生成围栏b
    PointXYZ fencea0 = plane_cloud->points[0];
    PointXYZ fencea1 = plane_cloud->points[1];
    PointXYZ fencea2 = plane_cloud->points[2];
    PointXYZ fencea3 = plane_cloud->points[3];
    drawPlane(appendFilter, fencea0, fencea1, fencea2, fencea3);
    appendFilter->Update();

    PointXYZ fenceb0 = plane_cloud->points[4];
    PointXYZ fenceb1 = plane_cloud->points[5];
    PointXYZ fenceb2 = plane_cloud->points[6];
    PointXYZ fenceb3 = plane_cloud->points[7];
    drawPlane(appendFilter, fenceb0, fenceb1, fenceb2, fenceb3);
    appendFilter->Update();


    vtkSmartPointer<vtkPolyData> polydata =
        vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);

    vtkSmartPointer<vtkDelaunay3D> delaunay =
        vtkSmartPointer<vtkDelaunay3D>::New();
    delaunay->SetInputData(polydata);
    delaunay->SetAlpha(0.15);
    delaunay->Update();


    vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter =
        vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
    surfaceFilter->SetInputData(delaunay->GetOutput());
    surfaceFilter->Update();

    vtkSmartPointer<vtkPolyData> polydata2 =
        vtkSmartPointer<vtkPolyData>::New();
     polydata2 = surfaceFilter->GetOutput();


    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polydata);
    glyphFilter->Update();


    vtkElevationFilter* ELFilter = vtkElevationFilter::New();
    ELFilter->SetInputData(polydata2);
    ELFilter->SetHighPoint(0, out_cloud->points[record_maxy].y, 0);
    ELFilter->SetLowPoint(0, out_cloud->points[record_miny].y, 0);
    ELFilter->SetScalarRange(1,0);


    vtkElevationFilter* mesh_Filter = vtkElevationFilter::New();
    mesh_Filter->SetInputConnection(appendFilter->GetOutputPort());
    mesh_Filter->Update();

    polyDataGlobe->DeepCopy(appendFilter->GetOutput());


    vtkSmartPointer<vtkPolyDataMapper> pointsMapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    pointsMapper->SetInputConnection(ELFilter->GetOutputPort());


    vtkSmartPointer<vtkPolyDataMapper> planeMapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    planeMapper->SetInputData(polyDataGlobe);


    vtkSmartPointer<vtkActor> pointsActor =
        vtkSmartPointer<vtkActor>::New();
    pointsActor->SetMapper(pointsMapper);


    vtkSmartPointer<vtkPolyDataMapper> triangulatedMapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    triangulatedMapper->SetInputConnection(ELFilter->GetOutputPort());


    vtkSmartPointer<vtkPolyDataMapper> fire_Mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    fire_Mapper->SetInputData(trans_Filter->GetOutput());


    vtkSmartPointer<vtkPolyDataMapper> fire2_Mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    fire2_Mapper->SetInputData(trans_Filter2->GetOutput());


    vtkSmartPointer<vtkActor> triangulatedActor =
        vtkSmartPointer<vtkActor>::New();
    triangulatedActor->SetMapper(triangulatedMapper);

    vtkSmartPointer<vtkActor> fire_Actor =
        vtkSmartPointer<vtkActor>::New();
    fire_Actor->SetMapper(fire_Mapper);
    fire_Actor->GetProperty()->SetColor(1, 0, 0);

    vtkSmartPointer<vtkActor> fire2_Actor =
        vtkSmartPointer<vtkActor>::New();
    fire2_Actor->SetMapper(fire2_Mapper);
    fire2_Actor->GetProperty()->SetColor(1, 0, 0);

    vtkSmartPointer<vtkActor> plane_Actor =
        vtkSmartPointer<vtkActor>::New();
    plane_Actor->SetMapper(planeMapper);
    plane_Actor->GetProperty()->SetColor(0, 1, 0);


    vtkSmartPointer<vtkRenderer> renderer =
        vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkCubeAxesActor> cubeAxesActor = vtkSmartPointer<vtkCubeAxesActor>::New();
    cubeAxesActor->SetCamera(renderer->GetActiveCamera());
    cubeAxesActor->SetBounds(polydata2->GetBounds());
    cubeAxesActor->SetXAxisRange(0, 5);
    cubeAxesActor->SetYAxisRange(0, 5);
    cubeAxesActor->SetZAxisRange(0, 5);
    cubeAxesActor->SetScreenSize(6);
    cubeAxesActor->SetLabelOffset(5);
    cubeAxesActor->SetVisibility(true);
    cubeAxesActor->SetFlyMode(0);
    cubeAxesActor->DrawXGridlinesOn();
    cubeAxesActor->DrawYGridlinesOn();
    cubeAxesActor->DrawZGridlinesOn();
    cubeAxesActor->SetDrawXInnerGridlines(false);
    cubeAxesActor->SetDrawYInnerGridlines(false);
    cubeAxesActor->SetDrawZInnerGridlines(false);
    cubeAxesActor->GetXAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    cubeAxesActor->GetYAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    cubeAxesActor->GetZAxesGridlinesProperty()->SetColor(0.5, 0.5, 0.5);
    cubeAxesActor->SetGridLineLocation(2);
    cubeAxesActor->XAxisMinorTickVisibilityOff();
    cubeAxesActor->YAxisMinorTickVisibilityOff();
    cubeAxesActor->ZAxisMinorTickVisibilityOff();
    cubeAxesActor->SetLabelScaling(false, 0, 0, 0);
    cubeAxesActor->SetTickLocation(1);


    renderer->AddActor(plane_Actor);
    renderer->AddActor(cubeAxesActor);
    renderer->AddActor(triangulatedActor);
    renderer->AddActor(fire_Actor);
    renderer->AddActor(fire2_Actor);
    renderer->GetActiveCamera()->Azimuth(40);
    renderer->GetActiveCamera()->Elevation(40);
    renderer->ResetCamera();
    renderer->ResetCameraClippingRange();
    renderer->SetBackground(0, 0, 0);

    vtkSmartPointer<vtkNamedColors> colors =
        vtkSmartPointer<vtkNamedColors>::New();

    vtkSmartPointer<vtkRenderWindow> renderWindow =
        vtkSmartPointer<vtkRenderWindow>::New();
    renderer->SetBackground(colors->GetColor3d("blue").GetData());
    renderWindow->AddRenderer(renderer);
//    renderWindow->SetSize(640, 640);
    //renderWindow->Render();
    ui->meshwidget->SetRenderWindow(renderWindow);
    ui->meshwidget->update();

    QString connect_s=" Connect the Lidar successful...";
    ui->label_9->setText(log_message(0,0,connect_s));
    ui->label_9->show();

    QString process_s=" Process pointcloud successful...";
    ui->label_9->setText(log_message(0,0,process_s));
    ui->label_9->show();

    ui->XFPutton->setStyleSheet("background-color: rgb(0,255,0);color:rgb(255,255,255);");
    ui->SFButton->setStyleSheet("background-color: rgb(0,255,0);color:rgb(255,255,255);");

    int TimerID1 = startTimer(10000);
    int TimerID2 = startTimer(20000);
    int TimerID3 = startTimer(30000);
    QString Flush=" start Flush...";
    ui->label_9->setText(log_message(0,0,Flush));
    ui->label_9->show();

}

void Widget::timerEvent(QTimerEvent * event )
{
    qDebug()<<"TimerID : "<<event->timerId();
    if(event->timerId()==3){
        QString Flush=" 30s Flush...";
        ui->label_9->setText(log_message(0,0,Flush));
        ui->label_9->show();

        //ui->warn_Button->setStyleSheet("background-color: rgb(255, 0,0);color:rgb(255,255,255);");
        //ui->stopButton->setStyleSheet("background-color: rgb(0, 255,0);color:rgb(255,255,255);");
        killTimer(event->timerId());
    }
    if (event->timerId()==4){
        QString Flush1=" 90s Flush...";
        ui->label_9->setText(log_message(0,0,Flush1));
        ui->label_9->show();
        QString warning=" Liquid warning...";
        ui->label_9->setText(log_message(0,0,warning));
        ui->label_9->show();
        ui->warn_Button->setStyleSheet("background-color: rgb(255, 0,0);color:rgb(255,255,255);");
        ui->stopButton->setStyleSheet("background-color: rgb(0, 255,0);color:rgb(255,255,255);");
        ui->XFPutton->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
        ui->SFButton->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
        QString close=" Auto close the firecannon and watervalve...";
        ui->label_9->setText(log_message(0,0,close));
        ui->label_9->show();
        killTimer(event->timerId());
    }
    if (event->timerId()==5){
        QString end_warning=" end the warning...";
        ui->label_9->setText(log_message(0,0,end_warning));
        ui->label_9->show();
        ui->warn_Button->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
        ui->stopButton->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
        ui->cycleButton->setStyleSheet("background-color: rgb(0, 255, 0);color:rgb(255,255,255);");
        killTimer(event->timerId());
    }
}


void Widget::on_stopButton_clicked(){
     ui->XFPutton->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
     ui->SFButton->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
     QString close=" Close the firecannon and watervalve...";
     show_anglepao(0,0,0,0,0,0,0,0);
     ui->label_9->setText(log_message(0,0,close));
     ui->label_9->show();
     ui->warn_Button->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
     ui->stopButton->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
}


void Widget::on_cycleButton_clicked(){
     show_anglepao(0,0,0,0,0,0,0,0);
      ui->cycleButton->setStyleSheet("background-color: rgb(170, 170, 127);color:rgb(255,255,255);");
}
