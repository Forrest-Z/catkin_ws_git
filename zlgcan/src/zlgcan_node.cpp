#include "ros/ros.h"
#include "control_manual/Cmd.h"
#include "pid/throttle.h"
#include "control_manual/manual_para.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "../include/zlgcan/controlcan.h"

bool manual_steer_control;
float steer_r;

#define msleep(ms)  usleep((ms)*1000)//定义进程挂起函数，单位：ms

unsigned gDevType = 4;//设备参数初始化
unsigned gDevIdx = 0;//设备序列号
unsigned gChMask = 1;//发送通道序号
unsigned gBaud = 0x1c00;//发送数据波特率
unsigned gTxType = 0;//发送模式////
unsigned gTxSleep = 1;//发送间隔时间////
unsigned gTxFrames = 100;//每次发送的帧数
unsigned gTxCount = 1;//

VCI_INIT_CONFIG config;//配置初始化
unsigned  CanIndex=gChMask;
VCI_CAN_OBJ vco[1];// 定义两帧的结构体数组

unsigned short int throtal,presure,steering,Degree_H[3],Degree_L[3];
unsigned short int DestSpeed;

void manualcmdCallback(const control_manual::manual_paraPtr& manual)
{
	manual_steer_control=manual->manual_steer_control;
	steer_r=manual->steer_r;//just for controlling the vehicle in manual,control steer by manual

  return;
}


void lonPlanCmdCallback(const pid::throttleConstPtr& lon_cmd){
	/*
        throtal     // 油门开度*100,范围0～100
	presure     // 制动压力*100，范围0～73
	steering    // 方向盘转角*1+900，范围0～1800
    */

    // by jyw
    //DestSpeed   = cmd->DestSpd;//***by hrq

    throtal = lon_cmd->throttle*100;
    presure = lon_cmd->pressure*100;
    //steering = cmd->DestStrAngle + 900;//***by hrq

        // NOTE: there is no throtal control 
	//throtal = 2;***by hrq

	//获取油门开度的高低位
	Degree_H[0]=throtal>>8;Degree_L[0]=throtal&(0x00ff);
	//获取制动压力的高低位
	Degree_H[1]=presure>>8;Degree_L[1]=presure&(0x00ff);
	//获取方向盘转角的高低位
	//Degree_H[2]=steering>>8;Degree_L[2]=steering&(0x00ff);//***by hrq

	vco[0].Data[0] = Degree_L[0];// 油门开度,低位
	vco[0].Data[1] = Degree_H[0];
	vco[0].Data[2] = Degree_L[1];// 制动压力，低位
	vco[0].Data[3] = Degree_H[1];
	/*vco[0].Data[4] = Degree_L[2];// 方向盘转角，低位
	vco[0].Data[5] = Degree_H[2];*///***by hrq

    return;
}
void PlanCmdCallback(const control_manual::CmdConstPtr& cmd){

if(manual_steer_control){

	steering=steer_r;	
}//control the steer by manual*/
else{
   steering = cmd->DestStrAngle + 900;//***by hrq
}
//}

        // NOTE: there is no throtal control 
	//throtal = 2;***by hrq

	/*//获取油门开度的高低位
	Degree_H[0]=throtal>>8;Degree_L[0]=throtal&(0x00ff);
	//获取制动压力的高低位
	Degree_H[1]=presure>>8;Degree_L[1]=presure&(0x00ff);*///***by hrq
	//获取方向盘转角的高低位
	Degree_H[2]=steering>>8;Degree_L[2]=steering&(0x00ff);

	/*vco[0].Data[0] = Degree_L[0];// 油门开度,低位
	vco[0].Data[1] = Degree_H[0];
	vco[0].Data[2] = Degree_L[1];// 制动压力，低位
	vco[0].Data[3] = Degree_H[1];*///***by hrq
	vco[0].Data[4] = Degree_L[2];// 方向盘转角，低位
	vco[0].Data[5] = Degree_H[2];

    return;
}

int zlgcan_init(){
	config.AccCode = 0;
	config.AccMask = 0xffffffff;
	config.Filter = 1;
	config.Mode = 0;
	config.Timing0 = gBaud & 0xff;
	config.Timing1 = gBaud >> 8;

	ROS_INFO("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d), TxCount=0x%08x(%d)\n",gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames, gTxFrames, gTxCount, gTxCount);

	if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) //判断设备是否打开
	{
		ROS_INFO("VCI_OpenDevice failed\n");
		return 0;
	}
	ROS_INFO("VCI_OpenDevice succeeded\n");
	
	if (!VCI_InitCAN(gDevType, gDevIdx, CanIndex, &config))//判断设备是否初始化完成
	{
		ROS_INFO("VCI_InitCAN(%d) failed\n", CanIndex);
		return 0;
	}
	ROS_INFO("VCI_InitCAN(%d) succeeded\n", CanIndex);

	if (!VCI_StartCAN(gDevType, gDevIdx, CanIndex))//判断设备已启动
	{
		ROS_INFO("VCI_StartCAN(%d) failed\n", CanIndex);
		return 0;
	}
	ROS_INFO("VCI_StartCAN(%d) succeeded\n", CanIndex);


	vco[0].ID = 0x00000050;// 填写第一帧的ID
	vco[0].SendType = gTxType;// 发送模式：正常发送
	vco[0].RemoteFlag = 0;// 数据帧
	vco[0].ExternFlag = 0;// 标准帧
	vco[0].DataLen = 6;// 数据长度6个字节

    return 0;
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "zlgcan_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
   
    zlgcan_init();
	
	//***********看
    ros::Subscriber manualcmd_sub       = nh.subscribe("manual_cmd", 1,  manualcmdCallback);
	ros::Subscriber throttle_under_pid_sub         = nh.subscribe("lon_under_pid", 1,  lonPlanCmdCallback);
    ros::Subscriber vehstate_sub         = nh.subscribe("planner_cmd", 1,  PlanCmdCallback);

    while (ros::ok){
        VCI_Transmit(gDevType, gDevIdx, CanIndex, vco, gTxFrames); // 发送gTxFrames帧
        ros::spinOnce();
        loop_rate.sleep();
    }
	//***********看

	ROS_INFO("\n entry <enter> to stop the transmit \n");
	getchar();
	VCI_CloseDevice(gDevType, gDevIdx);
	ROS_INFO("\n VCI_CloseDevice succeeded \n");

	return 0;
}
