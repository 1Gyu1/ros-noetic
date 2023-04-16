#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qnode.hpp"
#include "StatusDialog.hpp"
#include "CommonHeader.h"

namespace Ui {
class MainWindow;
}




class MainWindow : public QMainWindow
{
    Q_OBJECT

    systemCONFIG systemConfig;
    systemPOPUP  systemPopup;


    QByteArray recvBuf;
    QTimer timer;

    QTimer logicTimer;
    QTimer systemTimer;
    QTimer updateTimer;

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    QNode qnode;

    StatusDialog    *statusDialog;



    // Cobot Control API -------------------
    // <CobotInit>
    // : initialize Cobot
    void CobotInit();

    // <MoveJoint>
    // : move to target posture in joint coordinate
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1);

    // <MoveTCP>
    // : move to target posture in cartesian coordinate
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1);

    // <MoveCircle_ThreePoint>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current, first, and final position
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // x1, y1, z1 : first position in mm unit
    // rx1, ry1, rz1 : first orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // x2, y2, z2 : final position in mm unit
    // rx2, ry2, rz2 : final orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveCircle_ThreePoint(int type, float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd = -1, float acc = -1);

    // <MoveCircle_Axis>
    // : move current position to final position while it follows circle trajectory
    // : the circle trajectory is derived from current position, center position, axis, and rotation angle
    // type : 0 - try to follow both input position and orientation
    //        1 - orientation will be fixed to current orientation
    //        2 - orientation will be changed perpendicularly starting from current orientation
    // cx, cy, cz : center position in mm unit
    // ax, ay, az : axis representation (norminal)
    // rot_angle: rotation angle in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveCircle_Axis(int type, float cx, float cy, float cz, float ax, float ay, float az, float rot_angle, float spd, float acc);

    // <MoveJointBlend_Clear>
    // : clear joint blend list
    void MoveJointBlend_Clear();

    // <MoveJointBlend_AddPoint>
    // : add point to the joint blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // joint1~joint6 : target joint angle in deg unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveJointBlend_AddPoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1);

    // <MoveJointBlend_MovePoint>
    // : start to move all the points in the joint blend list
    void MoveJointBlend_MovePoint();

    // <MoveTCPBlend_Clear>
    // : clear TCP blend list
    void MoveTCPBlend_Clear();

    // <MoveTCPBlend_AddPoint>
    // : add point to the TCP blend list
    // : only the last point's 'vel' and 'acc' will be applied
    // radius : blend distance in mm unit
    // x, y, z : target TCP(tool center point) position in mm unit
    // rx, ry, rz : target TCP orientation (Yaw-Pitch-Roll Euler angle) in degree unit
    // spd : speed parameter (0~1: user define   or   -1: default setting)
    // acc : acceleration parameter (0~1: user define   or   -1: default setting)
    void MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1);

    // <MoveTCPBlend_MovePoint>
    // : start to move all the points in the joint blend list
    void MoveTCPBlend_MovePoint();

    // <ControlBoxDigitalOut>
    // control digital out ports in control box
    // d0~d15 : digital out value (0 or 1)
    void ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15);

    // <ControlBoxAnalogOut>
    // control analog out ports in control box
    // a0~a3 : analog out value in voltage unit (0~10)
    void ControlBoxAnalogOut(float a0, float a1, float a2, float a3);

    // <ToolOut>
    // control digital out ports and voltage level in tool flange board
    // volt : reference voltage of tool flange board in voltage unit(0, 12, 24)
    // d0, d1 : digital out value (0 or 1)
    void ToolOut(int volt, int d0, int d1);

    // <ProgramMode_Real>
    // change to 'real robot' mode -- robot will move
    void ProgramMode_Real();

    // <ProgramMode_Simulation>
    // change to 'simulation' mode -- robot will not move except teaching
    void ProgramMode_Simulation();

    // <BaseSpeedChange>
    // change base speed -- base speed will be multiplied to motion velocity
    // spd : normalized base speed (0~1)
    void BaseSpeedChange(float spd);

    // <MotionPause>
    // pause the current motion
    void MotionPause();

    // <MotionHalt>
    // halt the current motion
    // !! CAUTION : user would better escape the motion sequence
    //            : if not, the next motion will be activated immediately
    void MotionHalt();

    // <MotionResume>
    // resume the paused motion
    void MotionResume();

    // <CollisionResume>
    // resume the motion which is paused due to external collision
    void CollisionResume();
    // -------------------------------------




    int IsMotionIdle();

public Q_SLOTS:
    void onLogic();
    void onSystemCheck();
    void onUpdate();


private Q_SLOTS:
    void on_BTN_CONNECT_ROS_clicked();
    void on_BTN_COBOT_INIT_clicked();

    void on_BTN_MODE_REAL_clicked();
    void on_BTN_MODE_SIMULATION_clicked();

    void on_HS_BASE_SPEED_valueChanged(int value);
    void on_BTN_SPEED_CHANGE_clicked();

    void on_BTN_MOTION_PAUSE_clicked();
    void on_BTN_MOTION_RESUME_clicked();
    void on_BTN_MOTION_HALT_clicked();
    void on_BTN_COLLISION_RESUME_clicked();

    void on_BTN_GO_READY_clicked();
    void on_BTN_START_SCAN_clicked();

    void on_BTN_SEND_SCRIPT_clicked();

    void on_BTN_SEND_JOINT_POS_clicked();

private:
    Ui::MainWindow *ui;

    int initFlag;
};

#endif // MAINWINDOW_H
