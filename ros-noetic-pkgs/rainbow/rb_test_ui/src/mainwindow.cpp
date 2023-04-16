
#include "mainwindow.hpp"
#include "ui_mainwindow.h"

#include <QScrollBar>


systemSTAT systemStat;
int command_on_passing = false;
float joint_simulation[6];
float tcp_simulation[6];

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    qnode(argc,argv),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qnode.init();
    initFlag = false;

    memset(&systemStat, 0, sizeof(systemSTAT));
    systemStat.sdata.program_mode = -1;
    systemStat.sdata.robot_state = -1;

    statusDialog = new StatusDialog(ui->FRAME_STATUS);
    statusDialog->setWindowFlags(Qt::Widget);
    ui->FRAME_STATUS->setFixedSize(statusDialog->size());
    statusDialog->move(0,0);
    statusDialog->SetStatus(&systemStat);


    ui->SB_HEIGHT->setValue(0);
    ui->SB_RADIUS->setValue(40);


    connect(&systemTimer, SIGNAL(timeout()), this, SLOT(onSystemCheck()));
    connect(&logicTimer, SIGNAL(timeout()), this, SLOT(onLogic()));
    logicTimer.start(10);
    systemTimer.start(100);

    connect(&updateTimer, SIGNAL(timeout()), this, SLOT(onUpdate()));
    updateTimer.start(100);
}

MainWindow::~MainWindow()
{
    timer.stop();
    delete ui;
}


void MainWindow::on_BTN_CONNECT_ROS_clicked(){
    qnode.init();
}


void MainWindow::onSystemCheck(){

    // check initialize status
    if(initFlag == true){
        int init_info = systemStat.sdata.init_state_info;
        switch(init_info){
        case INIT_STAT_INFO_VOLTAGE_CHECK:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:red}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:red}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_DEVICE_CHECK:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:red}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_FIND_HOME:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_VARIABLE_CHECK:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:yellow}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:red}");
            break;
        case INIT_STAT_INFO_COLLISION_ON:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:yellow}");
            break;
        case INIT_STAT_INFO_INIT_DONE:
            ui->LE_INIT_POWER->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_DEVICE->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_SYSTEM->setStyleSheet("QLineEdit{background-color:green}");
            ui->LE_INIT_ROBOT->setStyleSheet("QLineEdit{background-color:green}");
            initFlag = false;
            break;
        case INIT_STAT_INFO_NOACT:
        default:
            break;
        }
    }
}


enum{
    BR_STATE_NON = 0,

    BR_STATE_GO_READY_1,    // move joint
    BR_STATE_GO_READY_2,    // move linear
    BR_STATE_GO_READY_3,    // move done

    BR_STATE_SCAN_1,    // move right 90
    BR_STATE_SCAN_2,    // move left 180
    BR_STATE_SCAN_3,    // move right 90
    BR_STATE_SCAN_4,    // move linear
    BR_STATE_SCAN_5,    // move up 45
    BR_STATE_SCAN_6,    // move down 90
    BR_STATE_SCAN_7,    // move up 45
    BR_STATE_SCAN_8     // move done
};
int test_flag = false;
int test_state = 0;

const float INI_POINT[6] = {200.0, 0.0, 450.0, -90.0, 0.0, -90.0};
float   RADIUS = 500.0;
float   HEIGHT = 0.0;
float   TOT_SPEED = 0.3;
float   TOT_ACC = 0.1;
void MainWindow::onLogic(){

    // Motion Sequence ===============================================
    if(test_flag == true){
        switch(test_state){
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        case BR_STATE_GO_READY_1:
            if(IsMotionIdle()){
                MoveJoint(33.60, -50.22, 152.87, -12.65, -89.99, 33.61, TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_GO_READY_2;
            }
            break;

        case BR_STATE_GO_READY_2:
            if(IsMotionIdle()){
                MoveTCP(INI_POINT[0], INI_POINT[1], INI_POINT[2]+HEIGHT, INI_POINT[3], INI_POINT[4], INI_POINT[5], TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_GO_READY_3;
            }
            break;

        case BR_STATE_GO_READY_3:
            if(IsMotionIdle()){
                test_state = BR_STATE_NON;
                test_flag = false;
            }
            break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        case BR_STATE_SCAN_1:
            if(IsMotionIdle()){
                MoveCircle_Axis(2, INI_POINT[0]+RADIUS, INI_POINT[1], INI_POINT[2]+HEIGHT, 0, 0, 1, -90.0, TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_SCAN_2;
            }
            break;
        case BR_STATE_SCAN_2:
            if(IsMotionIdle()){
                MoveCircle_Axis(2, INI_POINT[0]+RADIUS, INI_POINT[1], INI_POINT[2]+HEIGHT, 0, 0, 1, 180.0, TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_SCAN_3;
            }
            break;
        case BR_STATE_SCAN_3:
            if(IsMotionIdle()){
                MoveCircle_Axis(2, INI_POINT[0]+RADIUS, INI_POINT[1], INI_POINT[2]+HEIGHT, 0, 0, 1, -90.0, TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_SCAN_4;
            }
            break;
        case BR_STATE_SCAN_4:
            if(IsMotionIdle()){
                MoveTCP(INI_POINT[0]+50.0, INI_POINT[1], INI_POINT[2]+HEIGHT, INI_POINT[3], INI_POINT[4], INI_POINT[5], TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_SCAN_5;
            }
            break;
        case BR_STATE_SCAN_5:
            if(IsMotionIdle()){
                MoveCircle_Axis(2, INI_POINT[0]+RADIUS, INI_POINT[1], INI_POINT[2]+HEIGHT, 0, 1, 0, 45.0, TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_SCAN_6;
            }
            break;
        case BR_STATE_SCAN_6:
            if(IsMotionIdle()){
                MoveCircle_Axis(2, INI_POINT[0]+RADIUS, INI_POINT[1], INI_POINT[2]+HEIGHT, 0, 1, 0, -90.0, TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_SCAN_7;
            }
            break;
        case BR_STATE_SCAN_7:
            if(IsMotionIdle()){
                MoveCircle_Axis(2, INI_POINT[0]+RADIUS, INI_POINT[1], INI_POINT[2]+HEIGHT, 0, 1, 0, 45.0, TOT_SPEED, TOT_ACC);
                test_state = BR_STATE_SCAN_8;
            }
            break;
        case BR_STATE_SCAN_8:
            if(IsMotionIdle()){
                test_state = BR_STATE_NON;
                test_flag = false;
            }
            break;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }
    }
    // ===============================================================

}

void MainWindow::onUpdate(){


}

int MainWindow::IsMotionIdle(){
   return ((command_on_passing == false) && (systemStat.sdata.robot_state == 1));
}
void MainWindow::CobotInit(){
    QString text;
    text.sprintf("mc jall init");
    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::ProgramMode_Real(){
    QString text;
    text.sprintf("pgmode real");
    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::ProgramMode_Simulation(){
    QString text;
    text.sprintf("pgmode simulation");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc){
    QString text;
    text.sprintf("jointall %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd, float acc){
    QString text;
    text.sprintf("movetcp %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, x, y, z, rx, ry, rz);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveCircle_ThreePoint(int type, float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc){
    QString text;
    char buf[15];
    if(type == 0){
        sprintf(buf, "intended");
    }else if(type == 1){
        sprintf(buf, "constant");
    }else if(type == 2){
        sprintf(buf, "radial");
    }
    text.sprintf("movecircle absolute threepoints %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                 buf, spd, acc, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveCircle_Axis(int type, float cx, float cy, float cz, float ax, float ay, float az, float rot_angle, float spd, float acc){
    QString text;
    char buf[15];
    if(type == 0){
        sprintf(buf, "intended");
    }else if(type == 1){
        sprintf(buf, "constant");
    }else if(type == 2){
        sprintf(buf, "radial");
    }
    text.sprintf("movecircle absolute axis %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                 buf, spd, acc, rot_angle, cx, cy, cz, ax, ay, az);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveJointBlend_Clear(){
    QString text;
    text.sprintf("blend_jnt clear_pt");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveJointBlend_AddPoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc){
    QString text;
    text.sprintf("blend_jnt add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveJointBlend_MovePoint(){
    QString text;
    text.sprintf("blend_jnt move_pt");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveTCPBlend_Clear(){
    QString text;
    text.sprintf("blend_tcp clear_pt");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd, float acc){
    QString text;
    text.sprintf("blend_tcp add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, radius, x, y, z, rx, ry, rz);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MoveTCPBlend_MovePoint(){
    QString text;
    text.sprintf("blend_tcp move_pt");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15){
    QString text;
    text.sprintf("digital_out %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::ControlBoxAnalogOut(float a0, float a1, float a2, float a3){
    QString text;
    text.sprintf("analog_out %.3f, %.3f, %.3f, %.3f", a0, a1, a2, a3);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::ToolOut(int volt, int d0, int d1){
    int temp_volt = volt;
    if((temp_volt != 12) && (temp_volt != 24))
        temp_volt = 0;

    QString text;
    text.sprintf("tool_out %d, %d, %d", temp_volt, d0, d1);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::BaseSpeedChange(float spd){
    QString text;
    if(spd > 1.0)
        spd = 1.0;
    if(spd < 0.0)
        spd = 0.0;
    text.sprintf("sdw default_speed %.3f", spd);

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MotionPause(){
    QString text;
    text.sprintf("task pause");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MotionHalt(){
    QString text;
    text.sprintf("task stop");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::MotionResume(){
    QString text;
    text.sprintf("task resume_a");

    qnode.rb_command_publish(text.toStdString());
}
void MainWindow::CollisionResume(){
    QString text;
    text.sprintf("task resume_b");

    qnode.rb_command_publish(text.toStdString());
}






void MainWindow::on_BTN_COBOT_INIT_clicked(){
    CobotInit();
    initFlag = true;
}


void MainWindow::on_BTN_MODE_REAL_clicked(){
    ProgramMode_Real();
}

void MainWindow::on_BTN_MODE_SIMULATION_clicked(){
    ProgramMode_Simulation();
}

void MainWindow::on_HS_BASE_SPEED_valueChanged(int value){
    float spd = (float)value/100.0;
    ui->LB_BASE_SPEED->setText(QString().sprintf("%.1f%%", spd*100.0));
}
void MainWindow::on_BTN_SPEED_CHANGE_clicked(){
    float spd = (float)ui->HS_BASE_SPEED->value()/100.0;
    BaseSpeedChange(spd);
}


void MainWindow::on_BTN_MOTION_PAUSE_clicked(){
    MotionPause();
}

void MainWindow::on_BTN_MOTION_RESUME_clicked(){
    MotionResume();
}

void MainWindow::on_BTN_MOTION_HALT_clicked(){
    MotionHalt();
    // escape the motion sequence
    test_state = 0;
    test_flag = false;
}

void MainWindow::on_BTN_COLLISION_RESUME_clicked(){
    CollisionResume();
}

void MainWindow::on_BTN_GO_READY_clicked(){
    if(test_flag == true)
        return;

    RADIUS = ui->SB_RADIUS->value()*10.0;
    HEIGHT = ui->SB_HEIGHT->value()*10.0;

    test_state = BR_STATE_GO_READY_1;
    test_flag = true;
}

void MainWindow::on_BTN_START_SCAN_clicked(){
    if(test_flag == true)
        return;

    RADIUS = ui->SB_RADIUS->value()*10.0;
    HEIGHT = ui->SB_HEIGHT->value()*10.0;

    test_state = BR_STATE_SCAN_1;
    test_flag = true;
}



void MainWindow::on_BTN_SEND_SCRIPT_clicked(){

    qnode.rb_command_publish(ui->LE_SCRIPT->text().toStdString());
}



void MainWindow::on_BTN_SEND_JOINT_POS_clicked()
{
    double spd = 0.3;
    double acc = 0.1;
    double joint_pos[6];

    joint_pos[0] = ui->LE_SET_JNT_POS_1->text().toDouble();
    joint_pos[1] = ui->LE_SET_JNT_POS_2->text().toDouble();
    joint_pos[2] = ui->LE_SET_JNT_POS_3->text().toDouble();
    joint_pos[3] = ui->LE_SET_JNT_POS_4->text().toDouble();
    joint_pos[4] = ui->LE_SET_JNT_POS_5->text().toDouble();
    joint_pos[5] = ui->LE_SET_JNT_POS_6->text().toDouble();

    QString text;
    text.sprintf("jointall %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                 spd, acc,
                 joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]);

     qnode.rb_command_publish(text.toStdString());
}
