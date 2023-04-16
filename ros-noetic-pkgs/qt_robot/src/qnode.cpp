/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include "qnode.hpp"


extern systemSTAT systemStat;
extern int command_on_passing;
int ignore_cnt = 0;
extern float joint_simulation[6];
extern float tcp_simulation[6];

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qt_robot");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

    // publisher & subscriber ------------------------------
	ros::NodeHandle n;
    rb_command_pub = n.advertise<qt_robot::rb_command>("/rb_command", 1);
    rb_data_sub = n.subscribe("/rb_data", 10, rb_data_callback);
    // -----------------------------------------------------

	start();
	return true;
}

void QNode::rb_data_callback(const qt_robot::rb_data::ConstPtr &msg){
    if(ignore_cnt > 0){
        ignore_cnt--;
        command_on_passing = true;
    }else{
        command_on_passing = msg->command_on_passing;
    }


    systemStat.sdata.time = msg->time;

    for(int i=0; i<6; i++){
        systemStat.sdata.jnt_ref[i] = msg->joint_reference[i];
        systemStat.sdata.jnt_ang[i] = msg->joint_encoder[i];
        systemStat.sdata.cur[i] = msg->joint_current[i];
        systemStat.sdata.tcp_ref[i] = msg->tcp_reference[i];
        systemStat.sdata.temperature_mc[i] = msg->temperature[i];
        systemStat.sdata.jnt_info[i] = msg->joint_information[i];
        joint_simulation[i] = msg->joint_simulation[i];
        tcp_simulation[i] = msg->tcp_simulation[i];
    }

    systemStat.sdata.task_state = msg->task_state;
    systemStat.sdata.robot_state = msg->robot_state;
    systemStat.sdata.power_state = msg->power_state;
    systemStat.sdata.collision_detect_onoff = msg->collision_detect;
    systemStat.sdata.is_freedrive_mode = msg->freedrive_mode;
    systemStat.sdata.program_mode = msg->program_mode;

    systemStat.sdata.op_stat_collision_occur = msg->op_stat_collision_occur;
    systemStat.sdata.op_stat_sos_flag = msg->op_stat_sos_flag;
    systemStat.sdata.op_stat_self_collision = msg->op_stat_self_collision;
    systemStat.sdata.op_stat_soft_estop_occur = msg->op_stat_soft_estop_occur;
    systemStat.sdata.op_stat_ems_flag = msg->op_stat_ems_flag;

    for(int i=0; i<4; i++){
        systemStat.sdata.analog_in[i] = msg->analog_in[i];
        systemStat.sdata.analog_out[i] = msg->analog_out[i];
    }
    for(int i=0; i<16; i++){
        systemStat.sdata.digital_in[i] = msg->digital_in[i];
        systemStat.sdata.digital_out[i] = msg->digital_out[i];
    }
    for(int i=0; i<2; i++){
        systemStat.sdata.tfb_analog_in[i] = msg->tfb_analog_in[i];
        systemStat.sdata.tfb_digital_in[i] = msg->tfb_digital_in[i];
        systemStat.sdata.tfb_digital_out[i] = msg->tfb_digital_out[i];
    }
    systemStat.sdata.tfb_voltage_out = msg->tfb_voltage_out;

    systemStat.sdata.default_speed = msg->default_speed;



}

void QNode::rb_command_publish(std::string str){
    RB_COMMAND.header.stamp = ros::Time::now();
    RB_COMMAND.cmd = str;

    command_on_passing = true;
    ignore_cnt = 5;
    rb_command_pub.publish(RB_COMMAND);
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qt_robot");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

    // publisher & subscriber ------------------------------
    ros::NodeHandle n;
    rb_command_pub = n.advertise<qt_robot::rb_command>("/rb_command", 1);
    rb_data_sub = n.subscribe("/rb_data", 10, rb_data_callback);
    // -----------------------------------------------------

    start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(10);
	int count = 0;
    while ( ros::ok() ) {
		ros::spinOnce();
        loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


