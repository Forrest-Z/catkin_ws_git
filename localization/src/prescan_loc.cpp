/*
    sub vehstate from prescan /*订阅prescan发布的车辆状态信息，发布GPS IMU 消息
    pub GPS IMU msg
*/
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "localization/GPS.h"
#include "prescan/Vehstate.h"

const double RADIANS_PER_DEGREE = M_PI/180.0f;

ros::Publisher gps_pub, imu_pub;
localization::GPS global_position;
sensor_msgs::Imu IMU_data;

void EularAngleToQuaternion(double roll,double pitch,double yaw,double *qx,double *qy,double *qz,double *qw)
{
	double cosroll,sinroll,cospitch,sinpitch,cosyaw,sinyaw;
	cosroll=std::cos(roll/2);
	sinroll=std::sin(roll/2);

	cospitch=std::cos(pitch/2);
	sinpitch=std::sin(pitch/2);

	cosyaw=std::cos(yaw/2);
	sinyaw=std::sin(yaw/2);

	*qx=sinroll*cospitch*cosyaw-cosroll*sinpitch*sinyaw;
	*qy=cosroll*sinpitch*cosyaw+sinroll*cospitch*sinyaw;
	*qz=cosroll*cospitch*sinyaw-sinroll*sinpitch*cosyaw;
	*qw=cosroll*cospitch*cosyaw+sinroll*sinpitch*sinyaw;
}                                         //依据三个姿态角得出相应的四元数

void VehStateCallback(const prescan::Vehstate& vehstate)
{
    global_position.UTM_x = vehstate.x;
    global_position.UTM_y = vehstate.y;
    global_position.GPSSpd = vehstate.spd;//spd：速度
    global_position.GPSAcc = vehstate.acc;//acc：加速度

    // imu data
    double x,y,z,w, roll_angle,pitch_angle,yaw_angle;
    roll_angle = 0;
	pitch_angle= 0;
	yaw_angle  =  vehstate.heading;
	EularAngleToQuaternion(roll_angle,pitch_angle,yaw_angle,&x,&y,&z,&w);//调用上面定义的EularAngleToQuaternion函数
	IMU_data.orientation.x = x;
	IMU_data.orientation.y = y;
	IMU_data.orientation.z = z;
	IMU_data.orientation.w = w;
    return;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "prescan_loc");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Subscriber vehstate_sub = nh.subscribe("/vehstate", 1,  VehStateCallback);

    gps_pub = nh.advertise<localization::GPS> ("global_position", 1, true);
    imu_pub = nh.advertise<sensor_msgs::Imu> ("IMU_data", 1, true);



    while (ros::ok)
    {
        gps_pub.publish(global_position);
        imu_pub.publish(IMU_data);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
/*此程序实现了从prescan获取车辆状态信息转化为：车辆在全局坐标系下的GPS信息中的坐标、速度和加速度、
以及IMU中的角速度和角加速度（以四元数形式表示）发布出去*/