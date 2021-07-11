#include "ros/ros.h"
#include "pid/throttle.h"
#include "pid/velocity.h"
#include "control_manual/Cmd.h"
#include "prescan/Vehstate.h"//编译msg消息生成的头文件
#include "control_manual/manual_para.h"
#include "../include/pid/pidcontrol.h"
#include <stdio.h>
#include <math.h>
using namespace std;
pid::throttle lon_under_pid; //pid namespace中的throttle数据结构，lon_under_pid就是属于这种数据结构的变量，类似于 int a;--zrs
PIDController speed_pid_controller;//PIDController为类名，speed_pid_controller是属于这个类的变量，类似于 int a;--zrs
pid::velocity velocity_under_pid;
float v_r=0,v_r_original=0,v_r_1,delta_v_r,delta_v,v_o=0,error;
float v_r_manual,v_r_auto,v_r_1_manual,v_r_1_auto,v_r_o,v_r_1_o;
bool stop_cmd,manual_speed_control;
bool speed_up=true,speed_up_1=true;
historic_msg_throttle throttle;
historic_msg_pressure pressure;
void PlanCmdCallback(const control_manual::CmdPtr& cmd)//control_manual是功能包名，Cmd是此功能包下的msg名，cmd是Cmd类型的变量--zrs
{
   
    v_r_auto = cmd->DestSpd;
	v_r_1_auto=cmd->DestSpd_1;//control the vehicle by the planner

    return;
}
void VehStsCallback(const prescan::VehstatePtr& sts)
{
  v_o = sts->spd;

  return;
}

void manualcmdCallback(const control_manual::manual_paraPtr& manual)
{
    manual_speed_control=manual->manual_speed_control;//
	stop_cmd=manual->stop_cmd;
	v_r_manual=manual->v_r;
	v_r_1_manual=manual->v_r_1;//just for controlling the vehicle's vcelocity in manual

  return;
}


void control_init()
{
    PidConf parameters_pid;//PidConf是关于PID控制三个参数的数据结构，parameters_pid是属于此数据结构的变量--zrs

    parameters_pid.k_p_throttle=6;
    parameters_pid.k_i_throttle=0.0051;
    parameters_pid.k_d_throttle=0;//parameters of PID for the speed up at the status of uperring the basic velocity

    parameters_pid.k_p_pressure=8.0;
    parameters_pid.k_i_pressure=0;
    parameters_pid.k_d_pressure=0;//parameters of PID for the speed down at the status of uperring the basic velocity

	parameters_pid.k_p_pressure_up=12;
    parameters_pid.k_i_pressure_up=0.3;
    parameters_pid.k_d_pressure_up=0;//note:this value is revalued in "pidcontrol.cpp";//parameters of PID for the 
	                                                        //speed up at the status of lowerring the basic velocity

    parameters_pid.k_p_pressure_low=10;
    parameters_pid.k_i_pressure_low=0.55;
    parameters_pid.k_d_pressure_low=0;//note:this value is revalued in "pidcontrol.cpp";//parameters of PID for the 
	                                                        //speed down at the status of lowerring the basic velocity

    speed_pid_controller.Init(parameters_pid);/*调用speed_pid_controller中的成员函数Init()，此函数的作用是设置parameters_pid的九个参数，且清除the sum of error--zrs*/

    return;
}

void control(){

	if(manual_speed_control)//此语句的意思：如果 manual_speed_control为真（因为bool型的变量有两个值：true和false）则执行下面语句
	{

		v_r=v_r_manual;
		v_r_1=v_r_1_manual;//人工控制
	}
	else{

	    v_r=v_r_auto;
		v_r_1=v_r_1_auto;//算法控制
	}//control the vehcle by manual or the planner

	delta_v_r=v_r-v_r_1;
	delta_v=v_o-v_r;///////////////////////////////////////////////1.0

	speed_up_1=speed_up;

/*
	if(delta_v_r>=0){
		speed_up=true;
		ROS_INFO("SPEED UP,the previous velocity and present velocity is %f %f",v_r,v_r_1);
	}
	else if (delta_v_r<0){
		speed_up=false;
		ROS_INFO("SPEED down,the previous velocity and present velocity is %f %f",v_r,v_r_1);
	}*//////////////////////////////////////////////////////////////1.0

	if(v_r<=1.089)//如果目标速度小于等于1.089m/s，1.089m/s为车辆的带速，就是离合速度，如果车辆没有油门和刹车，即车辆会以带速行驶--zrs	////////////////////////////////////////////////1.0
{
		if(v_o-v_r<=0.1)/*0.1这个值是为了保持车辆的稳定性，同理见以下115行*/
		{
			speed_up=true;//加速
			ROS_INFO("SPEED UP OR NO SPEED UP OR SPEED DOWN,the target velocity and present velocity is %f %f",v_r,v_o);
		}
		else{
			speed_up=false;//减速
			ROS_INFO("SPEED DOWN OR NO SPEED UP OR SPEED DOWN,the target velocity and present velocity is %f %f",v_r,v_o);
		}
	}
	else{
		if(v_o-v_r<=0.5)/*为什么是0.5呢，因为目标速度大于1.089,可以适当的让这个误差项变大--zrs*/
		{
			speed_up=true;
			ROS_INFO("SPEED UP OR NO SPEED UP OR SPEED DOWN,the target velocity and present velocity is %f %f",v_r,v_o);
		}
		else{
			speed_up=false;
			ROS_INFO("SPEED DOWN OR NO SPEED UP OR SPEED DOWN,the target velocity and present velocity is %f %f",v_r,v_o);
		}
	}////////////////////////////////////////////////////////////////////////1.0
			

	v_r_original=v_r;//v_r_original相当于一个中间值，用于暂时保存v_r的值--zrs
	error=v_r-v_o;

	if(!speed_up&&v_r_original<=1.089&&v_o>1.089){
		v_r=2.0;
		if(v_o<=2.0)
		v_r=v_r_original;//如果车速大于2m/s时，使车辆不会一下子降速到1.089以下
	}


	if(!stop_cmd){

		if(speed_up_1!=speed_up)//如果上一个时刻的加速状态和此时刻的加速状态不相同--zrs
		{//declear the value of error_sum
			control_init();
			ROS_INFO("speed's mode transmited");
		}

		if(v_r>=1.089){//high speed

			if(speed_up){//speed up

				pressure.pressure=0;

				throttle = speed_pid_controller.Control_throttle(error,v_r);//throttle为一个变量名，speed_pid_controller.Control_throttle(error,v_r)返回值也为一个变量名

			        	if(throttle.throttle>=100.0)
						throttle.throttle=100.0;
       					else if(throttle.throttle<=0)
							throttle.throttle=0;//***by hrq:the value of throttle is 0~100

				lon_under_pid.throttle=throttle.throttle;
				lon_under_pid.pressure=pressure.pressure;
			}
			else//speed down
			{
				throttle.throttle=0;

        		pressure = speed_pid_controller.Control_pressure(error,v_r);
				pressure.pressure=abs(pressure.pressure);//abs()函数的作用是整型数取绝对值

        			if(pressure.pressure>=73)
						pressure.pressure=73;
       				else if(pressure.pressure<=0)
						pressure.pressure=0;//***by hrq:the value of throttle is 0~73

        		lon_under_pid.pressure=pressure.pressure;
				lon_under_pid.throttle=throttle.throttle;

				if((v_o-v_r)<=0.3||(v_r-v_o)>=0){//**********

					pressure.pressure=0;

					throttle = speed_pid_controller.Control_throttle(error,v_r);

			        	if(throttle.throttle>=100.0)
							throttle.throttle=100.0;
       					else if(throttle.throttle<=0)
							throttle.throttle=0;//***by hrq:the value of throttle is 0~100

					lon_under_pid.throttle=throttle.throttle;
					lon_under_pid.pressure=pressure.pressure;
				}//*************may can be deleted
			}
		}



//
		else{//lower speed

			if(speed_up){//speed up

				throttle.throttle=0;

        		pressure = speed_pid_controller.Control_pressure_up(error,v_r);
                      //为什么此加速状态需要有刹车，因为V0小于Vr，然而Vr，V0都在带速1.089以下，所以如果要从V0升到Vr，需要将刹车松一点
        			if(pressure.pressure_up>=73)
						pressure.pressure_up=73;
       				else if(pressure.pressure_up<=0)
						pressure.pressure_up=0;//***by hrq:the value of throttle is 0~73
				
				lon_under_pid.throttle=throttle.throttle;
        		lon_under_pid.pressure=pressure.pressure_up;
			}

			else{//speed down

				throttle.throttle=0;

        		pressure = speed_pid_controller.Control_pressure_low(error,v_r);

        			if(pressure.pressure_low>=73)
						pressure.pressure_low=73;
       				else if(pressure.pressure_low<=0)
						pressure.pressure_low=0;//***by hrq:the value of pressure is 0~73
				
				lon_under_pid.throttle=throttle.throttle;
        		lon_under_pid.pressure=pressure.pressure_low;
			}

		}

	}


	else{//stop the car

		lon_under_pid.throttle=0;
		lon_under_pid.pressure=73;
		control_init();//设置九个pid参且清除the sum of error--zrs
		v_r_1=0;//the value of the previous velocity
		v_r=0;//init the value of the target velocity
	}
	return;
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "lon_control");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Subscriber vehcmd_sub       = nh.subscribe("planner_cmd", 1,  PlanCmdCallback);
    ros::Subscriber vehstate_sub     = nh.subscribe("/vehstate", 1,  VehStsCallback);
	//ros::Subscriber vehstate_sub     = nh.subscribe("veh_loc_frenet", 1,  VehStsCallback);
    ros::Subscriber manualcmd_sub       = nh.subscribe("manual_cmd", 1,  manualcmdCallback);
	
    ros::Publisher lon_pid_pub  = nh.advertise<pid::throttle> ("lon_under_pid", 1);
    ros::Publisher velocyty_pid_pub  = nh.advertise<pid::velocity> ("velocity_under_pid", 1);

    control_init();
	v_r_1=0;//the value of the previous velocity
	v_r=0;//init the value of the target velocity

    while (ros::ok)
    {
	
	control();

        velocity_under_pid.v_r=v_r;
        velocity_under_pid.v_o=v_o;
        velocity_under_pid.delta_v=(error/v_r)*100;

        velocity_under_pid.header.stamp = ros::Time::now();
        lon_under_pid.header.stamp = ros::Time::now();
		//lon_under_pid.throttle=throttle.throttle;
		//lon_under_pid.pressure=pressure.pressure;本来应该有这两个赋值的，但是这赋值已经在control（）中已经完成了

        velocyty_pid_pub.publish(velocity_under_pid);
        lon_pid_pub.publish(lon_under_pid);

        ros::spinOnce();
        loop_rate.sleep();
    }
        ros::spin();
	return 0;
}
