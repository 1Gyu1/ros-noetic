#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>

#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/ExecuteTrajectoryActionGoal.h"
#include "moveit_msgs/ExecuteTrajectoryGoal.h"
#include "moveit_msgs/MoveGroupActionResult.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "moveit_msgs/MoveGroupActionFeedback.h"
#include "moveit_msgs/MoveGroupFeedback.h"

#include "actionlib_msgs/GoalStatusArray.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionFeedback.h"
#include "control_msgs/JointTrajectoryControllerState.h"

#include "CommonHeader.h"
#include "rb_connector/rb_command.h"
#include "rb_connector/rb_data.h"

// for socket client
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include <QString>
#include <signal.h>
#include <sstream>

#include <fstream>
#include <iostream>

#define VERSION 20210422
#define RB_CMD_PORT 5000
#define RB_DATA_PORT 5001
#define RX_DATA_SIZE 1000

int connect_nonb(int sockfd, const struct sockaddr *saptr, socklen_t salen,
                 int nsec);
int CreateSocket_Data(const char *addr, int port);
int CreateSocket_Command(const char *addr, int port);
int Connect2Server_Data();
int Connect2Server_Command();

void *Thread_Data(void *);
void *Thread_Command(void *);

void UpdateJoint();
void UpdateRBData();

void signalHandler(int _signalNO);
void signalPipeHandler(int _signalNO);

int sock_data = 0;
int sock_command = 0;
struct sockaddr_in server_data;
struct sockaddr_in server_command;

pthread_t THREAD_T_DATA;
pthread_t THREAD_T_COMMAND;
int threadWorking_Data = false;
int threadWorking_Command = false;
int connectionStatus_Data = false;
int connectionStatus_Command = false;
int command_seq = 0;

systemSTAT systemStat;
systemCONFIG systemConfig;
systemPOPUP systemPopup;

const float D2Rf = 0.0174533;
const float R2Df = 57.2957802;

const int NO_OF_JNT = 6;
const std::string JNTNameList[NO_OF_JNT] = {"base", "shoulder", "elbow",
                                            "wrist1", "wrist2", "wrist3"};

std::string default_ip = "192.168.1.222";
std::string ip;

rb_connector::rb_data RB_DATA;
rb_connector::rb_command RB_COMMAND;

sensor_msgs::JointState joint_state;

ros::Publisher joint_pub;
ros::Publisher rb_data_pub;
ros::Publisher rb_command_pub;
ros::Publisher rviz_update_state_pub;
ros::Publisher action_feed_pub;

ros::Subscriber rb_command_sub;
ros::Subscriber moveit_plan_sub;

actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *as_;

std::vector<trajectory_msgs::JointTrajectoryPoint> planned_trajectory_points;

float moveit_excute_final_pos[6] = {999, 999, 999, 999, 999, 999};
int update_flag = -1;
int pause_flag = -1;

void rb_send_command(std::string cmd)
{
  if (connectionStatus_Command)
  {
    if (command_seq == 0)
    {
      RB_DATA.command_on_passing = true;
      write(sock_command, cmd.data(), cmd.size());
      command_seq = 1;
    }

    int success = false;

    while (1)
    {
      if (command_seq == 4)
      {
        success = true;

        break;
      }
    }

    if (success)
    {
      std::cout << "command send done" << std::endl;
    }

    command_seq = 0;
    RB_DATA.command_on_passing = false;
  }
}

void rb_command_callback(const rb_connector::rb_command::ConstPtr &msg)
{
  std::string cmd = msg->cmd;
  if (connectionStatus_Command)
  {
    if (command_seq == 0)
    {
      RB_DATA.command_on_passing = true;
      write(sock_command, cmd.data(), cmd.size());
      command_seq = 1;
    }

    int success = false;
    for (int i = 0; i < 2000; i++)
    {
      usleep(1000);
      if (command_seq == 4)
      {
        success = true;
        break;
      }
    }

    if (success)
    {
      std::cout << "command execute done" << std::endl;
    }
    command_seq = 0;
    RB_DATA.command_on_passing = false;
  }
}

void rb_command_publish(std::string str)
{
  RB_COMMAND.header.stamp = ros::Time::now();
  RB_COMMAND.cmd = str;

  rb_command_pub.publish(RB_COMMAND);
}

void moveit_plan_trajectory_callback(
    const moveit_msgs::DisplayTrajectory &msg)
{
  planned_trajectory_points = msg.trajectory[0].joint_trajectory.points;
}

void executeCB()
{
  control_msgs::FollowJointTrajectoryGoalConstPtr goal;

  if (as_->isNewGoalAvailable())
  {
    std::cout << "acceptNewGoal" << std::endl;
    goal = as_->acceptNewGoal();

    int vectorSize = planned_trajectory_points.size();

    if (vectorSize > 0)
    {
      QString text;
      text.sprintf("move_ros_j_clear()");
      rb_send_command(text.toStdString());

      for (int i = 0; i < vectorSize; i++)
      {
        int time_sec = planned_trajectory_points[i].time_from_start.sec;
        int time_nsec = planned_trajectory_points[i].time_from_start.nsec;

        float real_time = ((float)time_sec) + ((float)time_nsec / 1000000000.);

        float pos[6];
        float vel[6];
        float acc[6];

        for (int j = 0; j < 6; j++)
        {
          pos[j] = planned_trajectory_points[i].positions[j] * R2Df;
          vel[j] = planned_trajectory_points[i].velocities[j] * R2Df;
          acc[j] = planned_trajectory_points[i].accelerations[j] * R2Df;
        }

        text.sprintf("move_ros_j_add(%.4f, jnt[%.3f, %.3f, %.3f, %.3f, %.3f, "
                     "%.3f], jnt[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f], "
                     "jnt[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f])",
                     real_time, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
                     vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], acc[0],
                     acc[1], acc[2], acc[3], acc[4], acc[5]);

        rb_send_command(text.toStdString());
      }

      for (int i = 0; i < 6; i++)
      {
        moveit_excute_final_pos[i] = planned_trajectory_points[vectorSize - 1].positions[i] * R2Df;
      }

      std::cout << "Trajectory size : " << vectorSize << std::endl;
      text.sprintf("move_ros_j_run(0,%d)", vectorSize);
      rb_send_command(text.toStdString());

      std::cout << "========== Trajectory transferd ==========  " << std::endl;
    }
  }
  else
  {
    std::cout << "NewGoal is not available" << std::endl;
  }
}

int main(int argc, char **argv)
{
  std::cout << "\033[1;32m===================================" << std::endl;
  std::cout << "   Now Start the RB Connetor..!!" << std::endl
            << std::endl;
  std::cout << "   Version  : " << VERSION << std::endl;
  std::cout << "   Developer: Jeongsoo Lim" << std::endl;
  std::cout << "   Developer: Taejune Kim" << std::endl;
  std::cout << "   E-mail   : jslim@rainbow-robotics.com" << std::endl;
  std::cout << "   E-mail   : june9473@rainbow-robotics.com" << std::endl;
  std::cout << "===================================\033[0m" << std::endl;

  ros::init(argc, argv, "rb_connector");

  signal(SIGINT, signalHandler);
  signal(SIGQUIT, signalHandler);
  signal(SIGABRT, signalHandler);
  signal(SIGKILL, signalHandler);
  signal(SIGHUP, signalHandler);
  signal(SIGPIPE, signalPipeHandler);

  // publisher & subscriber
  // ---------------------------------------------------------------------------
  ros::NodeHandle n;

  as_ = new actionlib::SimpleActionServer<
      control_msgs::FollowJointTrajectoryAction>(n, "/follow_joint_trajectory",
                                                 false);
  as_->registerGoalCallback(boost::bind(&executeCB));
  as_->start();

  n.param("rb_connector/ip", ip, default_ip);

  std::cout << "RB IP : " << ip.data() << std::endl;

  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);
  joint_state.name.resize(NO_OF_JNT);
  joint_state.position.resize(NO_OF_JNT);

  for (int i = 0; i < NO_OF_JNT; i++)
  {
    joint_state.name[i] = JNTNameList[i];
  }

  rb_command_pub = n.advertise<rb_connector::rb_command>("/rb_command", 10);
  rb_command_sub = n.subscribe("/rb_command", 10, rb_command_callback);

  rb_data_pub = n.advertise<rb_connector::rb_data>("/rb_data", 10);

  memset(&RB_DATA, 0, sizeof(RB_DATA));

  rviz_update_state_pub =
      n.advertise<std_msgs::Empty>("/rviz/moveit/update_start_state", 1);

  moveit_plan_sub = n.subscribe("/move_group/display_planned_path", 10,
                                moveit_plan_trajectory_callback);

  action_feed_pub =
      n.advertise<control_msgs::FollowJointTrajectoryActionFeedback>(
          "/follow_joint_trajectory/feedback", 10);
  // ---------------------------------------------------------------------------

  // Create Socket
  // ---------------------------------------------------------------------------
  if (CreateSocket_Data(ip.data(), RB_DATA_PORT))
  {
    int threadID = pthread_create(&THREAD_T_DATA, NULL, &Thread_Data, NULL);
    if (threadID < 0)
    {
      ROS_ERROR("Create Thread Error..(Data)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Create Socket Error..(Data)");
    return 0;
  }

  if (CreateSocket_Command(ip.data(), RB_CMD_PORT))
  {
    int threadID =
        pthread_create(&THREAD_T_COMMAND, NULL, &Thread_Command, NULL);
    if (threadID < 0)
    {
      ROS_ERROR("Create Thread Error..(Command)");
      return 0;
    }
  }
  else
  {
    ROS_ERROR("Create Socket Error..(Command)");
    return 0;
  }
  // ---------------------------------------------------------------------------

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    UpdateJoint();
    loop_rate.sleep();
  }

  if (shutdown(sock_data, SHUT_RDWR) == 0)
  {
    printf("rb_connector sock_data is shutdown.\n");
  }

  if (shutdown(sock_command, SHUT_RDWR) == 0)
  {
    printf("rb_connector sock_command is shutdown.\n");
  }

  return 0;
}

int CreateSocket_Data(const char *addr, int port)
{
  sock_data = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_data == -1)
  {
    return false;
  }

  server_data.sin_addr.s_addr = inet_addr(addr);
  server_data.sin_family = AF_INET;
  server_data.sin_port = htons(port);

  int optval = 1;
  setsockopt(sock_data, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  setsockopt(sock_data, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

  return true;
}
int CreateSocket_Command(const char *addr, int port)
{
  sock_command = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_command == -1)
  {
    return false;
  }
  server_command.sin_family = AF_INET;
  server_command.sin_addr.s_addr = inet_addr(addr);
  server_command.sin_port = htons(port);

  int optval = 1;
  setsockopt(sock_command, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  setsockopt(sock_command, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

  return true;
}

int Connect2Server_Data()
{
  int ret = 0;
  if ((ret = connect_nonb(sock_data, (struct sockaddr *)&server_data,
                          sizeof(server_data), 5)) < 0)
  {
    if (ret == -10)
    {
      close(sock_data);
      sock_data = 0;
    }
    return false;
  }
  int flags = fcntl(sock_data, F_GETFL, 0);
  fcntl(sock_data, F_SETFL, flags | O_NONBLOCK);

  std::cout << "Client connect to server!! (Data)" << std::endl;
  return true;
}
int Connect2Server_Command()
{
  int ret = 0;
  if ((ret = connect_nonb(sock_command, (struct sockaddr *)&server_command,
                          sizeof(server_command), 5)) < 0)
  {
    if (ret == -10)
    {
      close(sock_command);
      sock_command = 0;
    }
    return false;
  }
  int flags = fcntl(sock_command, F_GETFL, 0);
  fcntl(sock_command, F_SETFL, flags | O_NONBLOCK);

  std::cout << "Client connect to server!! (Command)" << std::endl;
  return true;
}

int test_cnt = 0;

void UpdateJoint()
{
  joint_state.header.stamp = ros::Time::now();

  for (int i = 0; i < NO_OF_JNT; i++)
  {
    joint_state.position[i] = systemStat.sdata.jnt_ref[i] * D2Rf;
  }

  joint_pub.publish(joint_state);

  // GoalID goal_id
  // uint8 status
  // uint8 PENDING         = 0   # The goal has yet to be processed by the action server
  // uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
  // uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
  //                             #   and has since completed its execution (Terminal State)
  // uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
  // uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due

  // RB_DATA.robot_state
  // 0 = None
  // 1 = Idle
  // 2 = Pause
  // 3 = Moving

  control_msgs::FollowJointTrajectoryActionFeedback actionFeedback;

  actionFeedback.header.stamp = ros::Time::now();

  if (RB_DATA.robot_state == 1)
  {
    actionFeedback.status.goal_id.id = "";
    actionFeedback.status.status = 3;
    actionFeedback.status.text = "IDLE";
    pause_flag = 0;
  }
  else if (RB_DATA.robot_state == 2)
  {
    actionFeedback.status.goal_id.id = "";
    actionFeedback.status.status = 2;
    actionFeedback.status.text = "PAUSE";
  }
  else if (RB_DATA.robot_state == 3)
  {
    actionFeedback.status.goal_id.id = "";
    actionFeedback.status.status = 1;
    actionFeedback.status.text = "MOVING";
    test_cnt++;

    if (pause_flag == 0 && as_->isActive() && as_->isPreemptRequested())
    {
      QString text;
      text.sprintf("task pause");
      rb_send_command(text.toStdString());

      text.sprintf("task stop");
      rb_send_command(text.toStdString());

      ROS_INFO("Preempted");
      as_->setPreempted();

      pause_flag = 1;
    }
  }

  actionFeedback.feedback.joint_names = {"base", "shoulder", "elbow",
                                         "wrist1", "wrist2", "wrist3"};

  sensor_msgs::JointState error_state;
  error_state.name.resize(NO_OF_JNT);
  error_state.position.resize(NO_OF_JNT);

  for (int i = 0; i < NO_OF_JNT; i++)
  {
    error_state.position[i] = 0.0f;
  }

  actionFeedback.feedback.actual.positions = joint_state.position;
  actionFeedback.feedback.desired.positions = joint_state.position;
  actionFeedback.feedback.error.positions = error_state.position;

  action_feed_pub.publish(actionFeedback);
}

void UpdateRBData()
{
  RB_DATA.header.stamp = ros::Time::now();
  RB_DATA.time = systemStat.sdata.time;
  for (int i = 0; i < 6; i++)
  {
    RB_DATA.joint_reference[i] = systemStat.sdata.jnt_ref[i];
    RB_DATA.joint_encoder[i] = systemStat.sdata.jnt_ang[i];
    RB_DATA.joint_current[i] = systemStat.sdata.cur[i];
    RB_DATA.tcp_reference[i] = systemStat.sdata.tcp_ref[i];
    RB_DATA.temperature[i] = systemStat.sdata.temperature_mc[i];
    RB_DATA.joint_information[i] = systemStat.sdata.jnt_info[i];
  }

  RB_DATA.task_state = systemStat.sdata.task_state;
  RB_DATA.robot_state = systemStat.sdata.robot_state;
  RB_DATA.power_state = systemStat.sdata.power_state;
  RB_DATA.collision_detect = systemStat.sdata.collision_detect_onoff;
  RB_DATA.freedrive_mode = systemStat.sdata.is_freedrive_mode;
  RB_DATA.program_mode = systemStat.sdata.program_mode;

  RB_DATA.op_stat_collision_occur = systemStat.sdata.op_stat_collision_occur;
  RB_DATA.op_stat_sos_flag = systemStat.sdata.op_stat_sos_flag;
  RB_DATA.op_stat_self_collision = systemStat.sdata.op_stat_self_collision;
  RB_DATA.op_stat_soft_estop_occur = systemStat.sdata.op_stat_soft_estop_occur;
  RB_DATA.op_stat_ems_flag = systemStat.sdata.op_stat_ems_flag;

  for (int i = 0; i < 4; i++)
  {
    RB_DATA.analog_in[i] = systemStat.sdata.analog_in[i];
    RB_DATA.analog_out[i] = systemStat.sdata.analog_out[i];
  }
  for (int i = 0; i < 16; i++)
  {
    RB_DATA.digital_in[i] = systemStat.sdata.digital_in[i];
    RB_DATA.digital_out[i] = systemStat.sdata.digital_out[i];
  }
  for (int i = 0; i < 2; i++)
  {
    RB_DATA.tfb_analog_in[i] = systemStat.sdata.tfb_analog_in[i];
    RB_DATA.tfb_digital_in[i] = systemStat.sdata.tfb_digital_in[i];
    RB_DATA.tfb_digital_out[i] = systemStat.sdata.tfb_digital_out[i];
  }
  RB_DATA.tfb_voltage_out = systemStat.sdata.tfb_voltage_out;

  RB_DATA.default_speed = systemStat.sdata.default_speed;

  rb_data_pub.publish(RB_DATA);

  // update moveit start state
  float err[6];

  for (int i = 0; i < 6; i++)
  {
    err[i] = moveit_excute_final_pos[i] - RB_DATA.joint_reference[i];
  }

  // check norm
  if (sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] +
           err[3] * err[3] + err[4] * err[4] + err[5] * err[5]) < 0.01)
  {
    update_flag = 1;
  }

  if (update_flag == 1)
  {
    std_msgs::Empty myMsg;
    rviz_update_state_pub.publish(myMsg);

    std::cout << "Update Start Pose" << std::endl;

    update_flag = 0;

    for (int i = 0; i < 6; i++)
    {
      moveit_excute_final_pos[i] = 999.0f;
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = 0;
    as_->setSucceeded(result, "Goal Reached");
  }
}

void *Thread_Data(void *)
{
  threadWorking_Data = true;

  unsigned int tcp_status = 0x00;
  int tcp_size = 0;
  int connectCnt = 0;
  int data_req_cnt = 0;

  static std::vector<unsigned char> totalData;
  static unsigned char recv_data[RX_DATA_SIZE];

  while (threadWorking_Data)
  {
    usleep(400);
    if (tcp_status == 0x00)
    {
      // If client was not connected
      if (sock_data == 0)
      {
        CreateSocket_Data(ip.data(), RB_DATA_PORT);
      }
      if (Connect2Server_Data())
      {
        tcp_status = 0x01;
        connectionStatus_Data = true;
        connectCnt = 0;

        totalData.clear();
      }
      else
      {
        if (connectCnt % 10 == 0)
        {
          std::cout << "Connect to Server Failed..(Data)" << std::endl;
        }
        connectCnt++;
      }
      usleep(1000 * 1000);
    }
    if (tcp_status == 0x01)
    {
      // If client was connected
      tcp_size = recv(sock_data, recv_data, RX_DATA_SIZE, 0);

      if (tcp_size == 0)
      {
        tcp_status = 0x00;
        connectionStatus_Data = false;
        close(sock_data);
        sock_data = 0;
        std::cout << "Socket Disconnected..(Data)" << std::endl;
      }
      else if (tcp_size > 0)
      {
        totalData.insert(totalData.end(), &recv_data[0], &recv_data[tcp_size]);

        while (totalData.size() > 4)
        {
          if (totalData[0] == '$')
          {
            int size = ((int)((unsigned char)totalData[2] << 8) |
                        (int)((unsigned char)totalData[1]));

            if (totalData.size() >= size)
            {
              if (totalData[3] == 3)
              {
                memcpy(&systemStat, totalData.data(), sizeof(systemSTAT));
                totalData.erase(totalData.begin(),
                                totalData.begin() + sizeof(systemSTAT));
                UpdateRBData();
                if (command_seq == 3)
                {
                  command_seq = 4;
                }
              }
              else if (totalData[3] == 4)
              {
                memcpy(&systemConfig, totalData.data(), sizeof(systemCONFIG));
                totalData.erase(totalData.begin(),
                                totalData.begin() + sizeof(systemCONFIG));
              }
              else if (totalData[4] == 10)
              {
                memcpy(&systemPopup, totalData.data(), sizeof(systemPOPUP));
                totalData.erase(totalData.begin(),
                                totalData.begin() + sizeof(systemPOPUP));
              }
              else
              {
                totalData.erase(totalData.begin(), totalData.begin() + 1);
              }
            }
          }
          else
          {
            totalData.erase(totalData.begin(), totalData.begin() + 1);
          }
        }
      }

      // request data -----
      data_req_cnt++;
      if (data_req_cnt % 500 == 0)
      {
        if (command_seq == 2)
        {
          write(sock_data, "reqdata\0", 8);
          command_seq = 3;
        }
        else
        {
          write(sock_data, "reqdata\0", 8);
        }
      }
    }
  }
  return NULL;
}

void *Thread_Command(void *)
{
  threadWorking_Command = true;

  unsigned int tcp_status = 0x00;
  int tcp_size = 0;
  int connectCnt = 0;

  static std::vector<unsigned char> totalData;
  static char recv_data[RX_DATA_SIZE];

  while (threadWorking_Command)
  {
    usleep(100);
    if (tcp_status == 0x00)
    {
      // If client was not connected
      if (sock_command == 0)
      {
        CreateSocket_Command(ip.data(), RB_CMD_PORT);
      }
      if (Connect2Server_Command())
      {
        tcp_status = 0x01;
        connectionStatus_Command = true;
        connectCnt = 0;
      }
      else
      {
        if (connectCnt % 10 == 0)
        {
          std::cout << "Connect to Server Failed..(Command)" << std::endl;
        }
        connectCnt++;
      }
      usleep(1000 * 1000);
    }

    if (tcp_status == 0x01)
    {
      // If client was connected
      tcp_size = recv(sock_command, recv_data, RX_DATA_SIZE, 0);
      if (tcp_size > 0)
      {
        std::string temp_str = recv_data;

        if (temp_str.compare("The command was executed\n") == 0)
        {
          std::cout << "[o] " << temp_str.data() << std::endl;

          command_seq = 4;
        }
        else
        {
          std::cout << "[x] " << temp_str.data() << std::endl;
        }
      }
    }
  }
  return NULL;
}

int connect_nonb(int sockfd, const struct sockaddr *saptr, socklen_t salen,
                 int nsec)
{
  int flags, n, error;
  socklen_t len;
  fd_set rset, wset;
  struct timeval tval;

  flags = fcntl(sockfd, F_GETFL, 0);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

  error = 0;
  if ((n = connect(sockfd, saptr, salen)) < 0)
  {
    if (errno != EINPROGRESS)
    {
      return (-10);
    }
  }

  /* Do whatever we want while the connect is taking place. */

  if (n == 0)
  {
    goto done; /* connect completed immediately */
  }

  FD_ZERO(&rset);
  FD_SET(sockfd, &rset);
  wset = rset;
  tval.tv_sec = nsec;
  tval.tv_usec = 0;

  if ((n = select(sockfd + 1, &rset, &wset, NULL, nsec ? &tval : NULL)) == 0)
  {
    close(sockfd); /* timeout */
    errno = ETIMEDOUT;
    return (-1);
  }

  if (FD_ISSET(sockfd, &rset) || FD_ISSET(sockfd, &wset))
  {
    len = sizeof(error);
    if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len) < 0)
    {
      return (-1); /* Solaris pending error */
    }
  }
  else
  {
    return -1;
    //        err_quit("select error: sockfd not set");
  }

done:
  fcntl(sockfd, F_SETFL, flags); /* restore file status flags */

  if (error)
  {
    close(sockfd); /* just in case */
    errno = error;
    return (-1);
  }
  return (0);
}

void signalHandler(int _signalNO)
{
  switch (_signalNO)
  {
  case SIGINT:
  case SIGQUIT:
  case SIGABRT:
  case SIGKILL:
  case SIGHUP:
    if (shutdown(sock_data, SHUT_RDWR) == 0)
    {
      printf("SIGNAL - rb_connector sock_data is shutdown.\n");
    }

    if (shutdown(sock_command, SHUT_RDWR) == 0)
    {
      printf("SIGNAL - rb_connector sock_command is shutdown.\n");
    }
    break;
  default:
    break;
  }
}

void signalPipeHandler(int _signalNO)
{
  printf("Caught signal SIGPIPE %d\n", _signalNO);
}
