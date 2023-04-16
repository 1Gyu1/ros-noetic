/**
 * @file /include/controll_robots/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef controll_robots_QNODE_HPP_
#define controll_robots_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include "CommonHeader.h"
#include "controll_robots/rb_data.h"
#include "controll_robots/rb_command.h"


class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    void rb_command_publish(std::string str);



Q_SIGNALS:
    void rosShutdown();

private:
    int init_argc;
    char** init_argv;


    controll_robots::rb_command  RB_COMMAND;

    ros::Publisher  rb_command_pub;
    ros::Subscriber rb_data_sub;

    static void rb_data_callback(const controll_robots::rb_data::ConstPtr &msg);

};


#endif /* controll_robots_QNODE_HPP_ */
