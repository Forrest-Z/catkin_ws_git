#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <control_manual/paraConfig.h>
#include "control_manual/manual_para.h"

control_manual::manual_para manual_cmd;
bool stop_cmd,manual_speed_control,manual_steer_control;
double v_r=0,steer_r,v_r_1;
void callback(control_manual::paraConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %s %s %s %f %f",
        config.manual_speed_control?"True":"False",
        config.manual_steer_control?"True":"False",
        config.stop_cmd?"True":"False",
        config.v_r,
        config.steer_r);

        //stop_cmd=config.stop_cmd?"True":"False";
        manual_speed_control=config.manual_speed_control;
        manual_steer_control=config.manual_steer_control;
	    stop_cmd=config.stop_cmd;
        steer_r=config.steer_r;

        v_r_1=v_r;

        v_r=config.v_r;
}
 
int main(int argc, char **argv)
{
    //ros::init(argc, argv, "examole5_dynamic_reconfigure");
    ros::init(argc, argv, "control_manual");

    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    dynamic_reconfigure::Server<control_manual::paraConfig> server;
    dynamic_reconfigure::Server<control_manual::paraConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Publisher manual_cmd_pub  = nh.advertise<control_manual::manual_para> ("manual_cmd", 1);


    while (ros::ok)
    {
        manual_cmd.manual_speed_control=manual_speed_control;
        manual_cmd.manual_steer_control=manual_steer_control;
        manual_cmd.stop_cmd=stop_cmd;
        manual_cmd.v_r=v_r;
        manual_cmd.steer_r=steer_r;

        manual_cmd.v_r_1=v_r_1;

        manual_cmd.header.stamp = ros::Time::now();

        manual_cmd_pub.publish(manual_cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::spin();
    return 0;
}
