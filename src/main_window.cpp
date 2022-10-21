/**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
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/class1_ros_qt_demo/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_ros_qt_demo {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
int frame_id=0;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    //需要连接硬件
    myCom = new QSerialPort();//这个在.h中定义了 QSerialPort *myCom;//声明对象
    myCom->setPortName("/dev/ttyUSB0");//串口的名称
    myCom->open(QIODevice::ReadWrite);//读写模式
    myCom->setBaudRate(QSerialPort::Baud115200);//波特率
    myCom->setBaudRate(QSerialPort::Data8);//8位数据
    myCom->setParity(QSerialPort::NoParity);//无校验位
    myCom->setStopBits(QSerialPort::OneStop);//1位停止
    myCom->setFlowControl(QSerialPort::NoFlowControl);//无数据流控制
    connect(myCom,SIGNAL(readyRead()),this,SLOT(slot_getSdata()));//连接接收的槽函数

    connect(ui.horizontalSlider_linerac,SIGNAL(valueChanged(int)),this,SLOT(slot_linear_value_change(int)));
    connect(ui.horizontalSlider_rawc,SIGNAL(valueChanged(int)),this,SLOT(slot_raw_value_change(int)));
    connect(ui.keyI,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.keyO,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.keyL,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.keyDot,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.keyDotc,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.keyM,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.keyJ,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.keyU,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));

    //terminal
    connect(ui.Enter_btn,SIGNAL(clicked()),this,SLOT(slot_quick_cmd_clicked()));

    //image1
    ui.lineEdit_image_topic->setText("/usb_cam/image_raw");
    ui.lineEdit_image_topic_2->setText("/image_raw");
    ui.lineEdit_image_topic_4->setText("/kinect2/sd/image_depth");///kinect2/sd/image_depth   /kinect2/qhd/image_mono
    ui.lineEdit_image_topic_3->setText("/kinect2/qhd/image_color");
    connect(ui.displayBtn,SIGNAL(clicked()),this,SLOT(solt_subImage()));
    connect(ui.displayBtn_2,SIGNAL(clicked()),this,SLOT(solt_subImage2()));



    //init ui dashBoard
    speedX_dashBoard=new CCtrlDashBoard(ui.widget_speedX);
    speedY_dashBoard=new CCtrlDashBoard(ui.widget_speedY);

    speedX_dashBoard->setGeometry(ui.widget_speedX->rect());
    speedY_dashBoard->setGeometry(ui.widget_speedY->rect());

    speedX_dashBoard->setValue(0);
    speedY_dashBoard->setValue(0);

    ui.horizontalSlider_linerac->setValue(30);
    ui.horizontalSlider_rawc->setValue(30);

    //header
    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
    ui.treeWidget->setHeaderHidden(true);

    //GLobal Options
    QTreeWidgetItem* Global=new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon("://images/options.png"));

    ui.treeWidget->addTopLevelItem(Global);
    Global->setExpanded(true);
    //FixFrame
    QTreeWidgetItem* Fixed_frame=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    fixed_box=new QComboBox();
    fixed_box->addItem("map");
    fixed_box->setMaximumWidth(150);
    fixed_box->setEditable(true);
    connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_value_change(QString)));
    Global->addChild(Fixed_frame);
    ui.treeWidget->setItemWidget(Fixed_frame,1,fixed_box);

    //Grid
    QTreeWidgetItem* Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    //设置图标
    Grid->setIcon(0,QIcon("://images/classes/Grid.png"));
    //checkbox
    QCheckBox* Grid_Check=new QCheckBox();
    connect(Grid_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_grid(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(Grid);
    //添加checkbox
    ui.treeWidget->setItemWidget(Grid,1,Grid_Check);
    //设置grid默认展开状态
    Grid->setExpanded(true);
    //添加Cell Count子节点
    QTreeWidgetItem* Cell_Count=new QTreeWidgetItem(QStringList()<<"Plane Cell Count");
    Grid->addChild(Cell_Count);
    //CellCount添加SpinBox
    Cell_Count_Box=new QSpinBox();
    Cell_Count_Box->setValue(13);
    //设置Spinbox的宽度
    Cell_Count_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Cell_Count,1,Cell_Count_Box);

    //添加color子节点
    QTreeWidgetItem* Grid_Color=new QTreeWidgetItem(QStringList()<<"Color");
    Grid->addChild(Grid_Color);
    //Color添加ComboBox
    Grid_Color_Box=new QComboBox();
    Grid_Color_Box->addItem("160;160;160");
    //设置Comboox可编辑
    Grid_Color_Box->setEditable(true);
    //设置Combox的宽度
    Grid_Color_Box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Grid_Color,1,Grid_Color_Box);

    //TF ui
    QTreeWidgetItem* TF=new QTreeWidgetItem(QStringList()<<"TF");
    //设置图标
    TF->setIcon(0,QIcon("://images/classes/TF.png"));
    //checkbox
    QCheckBox* TF_Check=new QCheckBox();
    connect(TF_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_tf(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(TF);
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(TF,1,TF_Check);

    //LaserScan
    QTreeWidgetItem* LaserScan=new QTreeWidgetItem(QStringList()<<"LaserScan");
    //设置图标
    LaserScan->setIcon(0,QIcon("://images/classes/LaserScan.png"));
    //checkbox
    QCheckBox* Laser_Check=new QCheckBox();
    connect(Laser_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_laser(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(LaserScan);
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(LaserScan,1,Laser_Check);
    //laser topic
    QTreeWidgetItem* LaserTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Laser_Topic_box=new QComboBox();
    Laser_Topic_box->addItem("/scan");
    Laser_Topic_box->setEditable(true);
    Laser_Topic_box->setMaximumWidth(150);
    LaserScan->addChild(LaserTopic);
    ui.treeWidget->setItemWidget(LaserTopic,1,Laser_Topic_box);

    //RobotModel
    QTreeWidgetItem* RobotModel=new QTreeWidgetItem(QStringList()<<"RobotModel");
    //设置图标
    RobotModel->setIcon(0,QIcon("://images/classes/RobotModel.png"));
    //checkbox
    QCheckBox* RobotModel_Check=new QCheckBox();
    connect(RobotModel_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_RobotModel(int)));
    //向Treewidget添加TF Top节点
    ui.treeWidget->addTopLevelItem(RobotModel);
    //向TF添加checkbox
    ui.treeWidget->setItemWidget(RobotModel,1,RobotModel_Check);


    //Map
    QTreeWidgetItem* Map=new QTreeWidgetItem(QStringList()<<"Map");
    //设置图标
    Map->setIcon(0,QIcon("://images/classes/Map.png"));
    //checkbox
    QCheckBox* Map_Check=new QCheckBox();
    connect(Map_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Map(int)));
    //向Treewidget添加Map Top节点
    ui.treeWidget->addTopLevelItem(Map);
    //向Map添加checkbox
    ui.treeWidget->setItemWidget(Map,1,Map_Check);
    //Map topic
    QTreeWidgetItem* MapTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Map_Topic_box=new QComboBox();
    Map_Topic_box->addItem("/map");
    Map_Topic_box->setEditable(true);
    Map_Topic_box->setMaximumWidth(150);
    Map->addChild(MapTopic);
    ui.treeWidget->setItemWidget(MapTopic,1,Map_Topic_box);
    //Map color scheme
    QTreeWidgetItem* MapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Map_Color_Scheme_box=new QComboBox();
    Map_Color_Scheme_box->addItem("map");
    Map_Color_Scheme_box->addItem("costmap");
    Map_Color_Scheme_box->addItem("raw");
    Map_Color_Scheme_box->setMaximumWidth(150);
    Map->addChild(MapColorScheme);
    ui.treeWidget->setItemWidget(MapColorScheme,1,Map_Color_Scheme_box);

    //Path
    QTreeWidgetItem* Path=new QTreeWidgetItem(QStringList()<<"Path");
    //设置图标
    Path->setIcon(0,QIcon("://images/classes/Path.png"));
    //checkbox
    QCheckBox* Path_Check=new QCheckBox();
    connect(Path_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Path(int)));
    //向Treewidget添加Path Top节点
    ui.treeWidget->addTopLevelItem(Path);
    //向Path添加checkbox
    ui.treeWidget->setItemWidget(Path,1,Path_Check);
    //Path topic
    QTreeWidgetItem* PathTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Path_Topic_box=new QComboBox();
    Path_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Path_Topic_box->setEditable(true);
    Path_Topic_box->setMaximumWidth(150);
    Path->addChild(PathTopic);
    ui.treeWidget->setItemWidget(PathTopic,1,Path_Topic_box);
    //Path color scheme
    QTreeWidgetItem* PathColorScheme=new QTreeWidgetItem(QStringList()<<"Color");
    Path_Color_box=new QComboBox();
    Path_Color_box->addItem("0;12;255");
    Path_Color_box->setEditable(true);
    Path_Color_box->setMaximumWidth(150);
    Path->addChild(PathColorScheme);
    ui.treeWidget->setItemWidget(PathColorScheme,1,Path_Color_box);

    //机器人Navigate 相关UI********************************
    //Golabal Map***************************************
    QTreeWidgetItem* GlobalMap=new QTreeWidgetItem(QStringList()<<"Global Map");
    GlobalMap->setIcon(0,QIcon("://images/default_package_icon.png"));
    QCheckBox* GlobalMap_Check=new QCheckBox();
    connect(GlobalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_global_map(int)));
    ui.treeWidget->addTopLevelItem(GlobalMap);
    ui.treeWidget->setItemWidget(GlobalMap,1,GlobalMap_Check);

    //Global CostMap
    QTreeWidgetItem* Global_CostMap=new QTreeWidgetItem(QStringList()<<"Costmap");
    //设置图标
    Global_CostMap->setIcon(0,QIcon("://images/classes/Map.png"));
    //Global Map添加子节点
    GlobalMap->addChild(Global_CostMap);
    //Map topic
    QTreeWidgetItem* Global_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Global_CostMap_Topic_box=new QComboBox();
    Global_CostMap_Topic_box->addItem("/move_base/global_costmap/costmap");
    Global_CostMap_Topic_box->setEditable(true);
    Global_CostMap_Topic_box->setMaximumWidth(150);
    Global_CostMap->addChild(Global_CostMap_Topic);
    ui.treeWidget->setItemWidget(Global_CostMap_Topic,1,Global_CostMap_Topic_box);
    //Map color scheme
    QTreeWidgetItem* GlobalMapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    GlobalMapColorScheme_box=new QComboBox();
    GlobalMapColorScheme_box->addItem("costmap");
    GlobalMapColorScheme_box->addItem("map");
    GlobalMapColorScheme_box->addItem("raw");
    GlobalMapColorScheme_box->setMaximumWidth(150);
    Global_CostMap->addChild(GlobalMapColorScheme);
    ui.treeWidget->setItemWidget(GlobalMapColorScheme,1,GlobalMapColorScheme_box);

    //Global Planner
    QTreeWidgetItem* Global_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
    //设置图标
    Global_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
    //向TGlobal Map添加Path Top节点
    GlobalMap->addChild(Global_Planner);

    //Path topic
    QTreeWidgetItem* Global_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Global_Planner_Topic_box=new QComboBox();
    Global_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/global_plan");
    Global_Planner_Topic_box->setEditable(true);
    Global_Planner_Topic_box->setMaximumWidth(150);
    Global_Planner->addChild(Global_Planner_Topic);
    ui.treeWidget->setItemWidget(Global_Planner_Topic,1,Global_Planner_Topic_box);
    //Path color scheme
    QTreeWidgetItem* Global_Planner_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color");
    Global_Planner_Color_box=new QComboBox();
    Global_Planner_Color_box->addItem("255;0;0");
    Global_Planner_Color_box->setEditable(true);
    Global_Planner_Color_box->setMaximumWidth(150);
    Global_Planner->addChild(Global_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget(Global_Planner_Color_Scheme,1,Global_Planner_Color_box);

    //Local Map***********************************************
    QTreeWidgetItem* LocalMap=new QTreeWidgetItem(QStringList()<<"Local Map");
    LocalMap->setIcon(0,QIcon("://images/default_package_icon.png"));
    QCheckBox* LocalMap_Check=new QCheckBox();
    connect(LocalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_local_map(int)));
    ui.treeWidget->addTopLevelItem(LocalMap);
    ui.treeWidget->setItemWidget(LocalMap,1,LocalMap_Check);

    //Local CostMap
    QTreeWidgetItem* Local_CostMap=new QTreeWidgetItem(QStringList()<<"Costmap");
    //设置图标
    Local_CostMap->setIcon(0,QIcon("://images/classes/Map.png"));
    //Local Map添加子节点
    LocalMap->addChild(Local_CostMap);
    //Map topic
    QTreeWidgetItem* Local_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_CostMap_Topic_box=new QComboBox();
    Local_CostMap_Topic_box->addItem("/move_base/local_costmap/costmap");
    Local_CostMap_Topic_box->setEditable(true);
    Local_CostMap_Topic_box->setMaximumWidth(150);
    Local_CostMap->addChild(Local_CostMap_Topic);
    ui.treeWidget->setItemWidget(Local_CostMap_Topic,1,Local_CostMap_Topic_box);
    //Map color scheme
    QTreeWidgetItem* LocalMapColorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    LocalMapColorScheme_box=new QComboBox();
    LocalMapColorScheme_box->addItem("costmap");
    LocalMapColorScheme_box->addItem("map");
    LocalMapColorScheme_box->addItem("raw");
    LocalMapColorScheme_box->setMaximumWidth(150);
    Local_CostMap->addChild(LocalMapColorScheme);
    ui.treeWidget->setItemWidget(LocalMapColorScheme,1,LocalMapColorScheme_box);

    //Local Planner
    QTreeWidgetItem* Local_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
    //设置图标
    Local_Planner->setIcon(0,QIcon("://images/classes/Path.png"));
    //向TLocal Map添加Path Top节点
    LocalMap->addChild(Local_Planner);

    //Path topic
    QTreeWidgetItem* Local_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_Planner_Topic_box=new QComboBox();
    Local_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Local_Planner_Topic_box->setEditable(true);
    Local_Planner_Topic_box->setMaximumWidth(150);
    Local_Planner->addChild(Local_Planner_Topic);
    ui.treeWidget->setItemWidget(Local_Planner_Topic,1,Local_Planner_Topic_box);
    //Path color scheme
    QTreeWidgetItem* Local_Planner_Color_Scheme=new QTreeWidgetItem(QStringList()<<"Color");
    Local_Planner_Color_box=new QComboBox();
    Local_Planner_Color_box->addItem("0;12;255");
    Local_Planner_Color_box->setEditable(true);
    Local_Planner_Color_box->setMaximumWidth(150);
    Local_Planner->addChild(Local_Planner_Color_Scheme);
    ui.treeWidget->setItemWidget(Local_Planner_Color_Scheme,1,Local_Planner_Color_box);



    //set start pose，target pos
    connect(ui.set_start_btn,SIGNAL(clicked()),this,SLOT(slot_set_start_pose()));
    connect(ui.set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_goal_pose()));
    connect(&qnode,SIGNAL(position(double,double,double)),this,SLOT(slot_update_pos(double,double,double)));

//    ui.checkGrisp->setC
    //connect
    connect(&qnode,SIGNAL(speed_vel(float,float)),this,SLOT(slot_updat_dashboard(float,float)));
    connect(&qnode,SIGNAL(mapTf(float,float,float,double,double,double)),this,SLOT(slot_update_tf(float,float,float,double,double,double)));
    //grisp
    connect(ui.grip_btnP,SIGNAL(pressed()),this,SLOT(slot_gripP()));
    connect(ui.grip_btnS,SIGNAL(clicked()),this,SLOT(slot_gripS()));
    connect(ui.horizontalScrollBar_grip,SIGNAL(valueChanged(int)),this,SLOT(slot_grip(int)));
    connect(ui.checkGrisp,SIGNAL(clicked(bool)),this,SLOT(slot_triggerGrisp()));
    //ur5
    connect(ui.btn_UR5,SIGNAL(clicked()),this,SLOT(slot_UR5()));





}
void MainWindow::slot_update_pos(double x,double y,double z)
{
    ui.pos_x->setText(QString::number(x));
    ui.pos_y->setText(QString::number(y));
    ui.pos_z->setText(QString::number(z));
}
void MainWindow::slot_display_local_map(int state)
{
      bool enable=state>1?true:false;
      QStringList qli=Local_Planner_Color_box->currentText().split(";");
      QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
      myrviz->Display_Local_Map(Local_CostMap_Topic_box->currentText(),LocalMapColorScheme_box->currentText(),Local_Planner_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_display_global_map(int state)
{
      bool enable=state>1?true:false;
      QStringList qli=Global_Planner_Color_box->currentText().split(";");
      QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
      myrviz->Display_Global_Map(Global_CostMap_Topic_box->currentText(),GlobalMapColorScheme_box->currentText(),Global_Planner_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_set_start_pose()
{
    myrviz->Set_Start_Pose();
}
void MainWindow::slot_set_goal_pose()
{
    myrviz->Set_Goal_Pose();
}
void MainWindow::slot_display_Path(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Path_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Path(Path_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_display_Map(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_Map(Map_Topic_box->currentText(),Map_Color_Scheme_box->currentText(),enable);
}
void MainWindow::slot_display_RobotModel(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_RobotModel(enable);
}
void MainWindow::slot_display_laser(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_LaserScan(Laser_Topic_box->currentText(),enable);
}
void MainWindow::slot_display_tf(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_TF(enable);
}


void MainWindow::slot_display_grid(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Grid_Color_Box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Grid(Cell_Count_Box->text().toInt(),color,enable);
}
void MainWindow::slot_treewidget_value_change(QString)
{
    myrviz->Set_FixedFrame(fixed_box->currentText());//目前值
}

void MainWindow::slot_triggerGrisp()
{
//    if(ui.checkGrisp->isChecked())
//    {
//        ui.horizontalScrollBar_grip->setValue(glovec.thumb);

}

void MainWindow::slot_getSdata()
{
    QByteArray buf_data;            //接收数据，
    QString strGripD;

    buf_data = myCom->readAll();    //接收数据
//    ui.serialRead->setText(buf_data);
    qDebug()<<buf_data.size();
    qDebug()<<buf_data<<endl;
    if(!buf_data.isEmpty())
    {
//        if(buf_data.size()==12)
        QString strTemp=QString(buf_data);
        QString strGrip=strTemp.mid(strTemp.indexOf("\r\n"),8);
        strGripD=strGrip.mid(2,4);
//        qDebug()<<strGripD;
    }
    glovec.thumb=strGripD.toInt(&ok);
    glovec.thumb=glovec.thumb*0.22-66;//transform to scale 0-100
    if(ui.checkGrisp->isChecked())
    {
        ui.label_grip->setText(QString::number(glovec.thumb));
        qnode.gripChange(glovec.thumb*2.55);
    }
    qDebug()<<glovec.thumb;

   QThread::sleep(0.01);

//    qWarning()<<buf_data.at(3);
    //ui->ReceiveMsg_Lie->setText("");
}
void MainWindow::slot_gripP()
{
    qnode.gripP();
}
void MainWindow::slot_UR5()
{
    Ur5=new QProcess;
    QString prog="~/qt_ros/src/UR5/ur5Linux/UR5";//absoluted path
    Ur5->start(prog);
}

void MainWindow::slot_gripS()
{
    qnode.gripS();
}
void MainWindow::slot_grip(int v)
{
    ui.label_grip->setText(QString::number(v));
    qDebug()<<v*2.55;
    qnode.gripChange(v*2.55);
}

void MainWindow::slot_updat_dashboard(float x,float y)
{
    ui.label_dirx->setText(x>0?"+f":"-");
    ui.label_diry->setText(y>0?"+":"-");
    speedX_dashBoard->setValue(abs(x*100));
    speedY_dashBoard->setValue(abs(y*100));

}
void MainWindow::slot_update_tf(float x,float y,float z,double pitch,double roll,double yaw)
{
    ui.label_15->setNum(x);
    ui.label_16->setNum(y);
    ui.label_17->setNum(z);
    ui.label_18->setNum(pitch);
    ui.label_19->setNum(roll);
    ui.label_20->setNum(yaw);
}

void MainWindow::slot_update_image(int id,QImage im)
{
    qimage_mutex_.lock();
    qDebug()<<"show image1";
//    switch(id)
//    {
//     case 0:
//        ui.labelImage->setPixmap(QPixmap::fromImage(im).scaled(ui.labelImage->width(),ui.labelImage->height()));
//        break;
//     case 1:
//        ui.labelImage_2->setPixmap(QPixmap::fromImage(im).scaled(ui.labelImage_2->width(),ui.labelImage_2->height()));
//        break;
//    }
   // ui.labelImage->setPixmap(QPixmap:: fromImage(im).scaled(ui.labelImage->width(),ui.labelImage->height()));
//ui.labelImage_2->setPixmap(QPixmap::fromImage(im).scaled(ui.labelImage_2->width(),ui.labelImage_2->height()));
    ui.labelImage->setPixmap(QPixmap::fromImage(im).scaled(ui.labelImage->width(),ui.labelImage->height()));
    qimage_mutex_.unlock();


}
void MainWindow::slot_update_image2(int id,QImage im)
{
    qimage_mutex_2.lock();
    qDebug()<<"show image2";
     ui.labelImage_2->setPixmap(QPixmap::fromImage(im).scaled(ui.labelImage_2->width(),ui.labelImage_2->height()));
    qimage_mutex_2.unlock();
}
void MainWindow::slot_update_image3(int id,QImage im)
{
    qimage_mutex_3.lock();
    //qDebug()<<"show image3";
     ui.labelImage_3->setPixmap(QPixmap::fromImage(im).scaled(ui.labelImage_3->width(),ui.labelImage_3->height()));
    qimage_mutex_3.unlock();
}
void MainWindow::slot_update_image4(int id,QImage im)
{
    qdepth_mutex_.lock();
    //qDebug()<<"show image4";
     ui.labelImage_4->setPixmap(QPixmap::fromImage(im).scaled(ui.labelImage_4->width(),ui.labelImage_4->height()));
    qdepth_mutex_.unlock();
}
void MainWindow::initVideos()
{
//    if(ui.lineEdit_image_topic->text()!="")
//    if(ui.lineEdit_image_topic_2->text()!="")
//    qnode.subImage2(ui.lineEdit_image_topic->text(),0);
   // qnode.subImage(ui.lineEdit_image_topic_2->text(),2);
    //qnode.subImage2(ui.lineEdit_image_topic->text(),0);
    qnode.subImage3(ui.lineEdit_image_topic_3->text(),1);
    qnode.subImage4(ui.lineEdit_image_topic_4->text(),3);//no 3
    qnode.subFindTf("objectsStamped");
   // connect(&qnode,SIGNAL(image_vel(int,QImage)),this,SLOT(slot_update_image(int,QImage)));
   // connect(&qnode,SIGNAL(image_vel2(int,QImage)),this,SLOT(slot_update_image2(int,QImage)));
    connect(&qnode,SIGNAL(image_vel3(int,QImage)),this,SLOT(slot_update_image3(int,QImage)));
    connect(&qnode,SIGNAL(image_vel4(int,QImage)),this,SLOT(slot_update_image4(int,QImage)));

}

void MainWindow::solt_subImage()
{
    qnode.subImage(ui.lineEdit_image_topic->text(),0);
}
void MainWindow::solt_subImage2()
{
    frame_id=1;
    qnode.subImage2(ui.lineEdit_image_topic_2->text(),2);

}
void MainWindow::slot_quick_cmd_clicked()
{
    cmd=new QProcess;
    cmd->start("bash");
    cmd->write(ui.textEditCmd->toPlainText().toLocal8Bit()+'\n');
    connect(cmd,SIGNAL(readyReadStandardError()),this,SLOT(slot_quick_cmd_output()));
    connect(cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_cmd_output()));
}
void MainWindow::slot_quick_cmd_output()
{
    ui.textEditInfo->append("<font color=\"#FF0000\">"+cmd->readAllStandardError()+"</font>");
    ui.textEditInfo->append("<font color=\"#FFFFFF\">"+cmd->readAllStandardOutput()+"</font>");

}

void MainWindow::slot_pushbtn_click()
{
    QPushButton* btn=qobject_cast<QPushButton*>(sender());
    qDebug()<<btn->text();
    char key=btn->text().toStdString()[0];//transform data type
    bool is_all=ui.AnyCheck->isChecked();
    float linear=ui.label_linerac->text().toFloat()*0.01;
    float angular=ui.label_rawc->text().toFloat()*0.01;
    switch(key){
    case 'i':
        qnode.set_cmd_vel(is_all?'I':'i',linear,angular);
        break;
    case 'u':
        qnode.set_cmd_vel(is_all?'U':'u',linear,angular);
        break;
      case 'o':
        qnode.set_cmd_vel(is_all?'O':'o',linear,angular);
        break;
      case 'j':
        qnode.set_cmd_vel(is_all?'J':'j',linear,angular);
        break;
      case 'l':
        qnode.set_cmd_vel(is_all?'L':'l',linear,angular);
        break;
      case 'm':
        qnode.set_cmd_vel(is_all?'M':'m',linear,angular);
        break;
      case ',':
        qnode.set_cmd_vel(is_all?'<':',',linear,angular);
        break;
      case '.':
        qnode.set_cmd_vel(is_all?'>':'.',linear,angular);
        break;


    }
}

void MainWindow::slot_linear_value_change(int value)
{
    ui.label_linerac->setText(QString::number(value));
}
void MainWindow::slot_raw_value_change(int value)
{
    ui.label_rawc->setText(QString::number(value));
}
MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {//use enviroment
        if ( !qnode.init() ) {
            showNoMasterMessage();
            ui.treeWidget->setEnabled(false);
        } else {
            ui.button_connect->setEnabled(false);
                initVideos();
            //rviz 初始化
            ui.treeWidget->setEnabled(true);
            myrviz=new qrviz(ui.layout_rviz_widget);    
                qDebug()<<"tr2";
        }
    } else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                   ui.line_edit_host->text().toStdString()) ) {
            showNoMasterMessage();
            //rviz 初始化
            ui.treeWidget->setEnabled(false);
        } else {

            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
            initVideos();
            //rviz 初始化
             ui.treeWidget->setEnabled(true);
            myrviz=new qrviz(ui.layout_rviz_widget);
            qDebug()<<"tr1";
        }
    }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
//    initVideos();
//    qDebug()<<"tr2";
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_ros_qt_demo");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace class1_ros_qt_demo

