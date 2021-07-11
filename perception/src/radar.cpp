#include "ros/ros.h"
#include "iostream"
#include "math.h"

#include "prescan/Vehradar.h"
#include "perception/RadarObstacles.h"
#include "../include/perception/radarsetup.h"

perception::RadarObstacles radarobstacles;

double deg2rad(double x) { return x * M_PI / 180; } //deg2rad（）为角度转化为弧度的函数

void RadarFront_Callback( const prescan::VehradarConstPtr& radar_front ){
    for (int i = 0; i < radar_front->ranges.size(); i++ ){//遍历前方雷达传感器检测到的所有障碍物
        perception::RadarObstacle obstacle;
        if ( radar_front->ranges[i] > 0 ){
            // theta here means the relative angle to radar sensor
            // it is different from the ObstacleFrenet
            double theta = deg2rad(radar_front->theta[i]);//障碍物方位角，与传感器坐标系x轴夹角，右边为正，左边为负
            // TODO: trans the spd
            obstacle.speed = radar_front->dopplervel[i];//障碍物相对于车辆的速度，方向沿着方位角方向
            // index - 1
            int obs_typeid = radar_front->type[i] - 1;// 障碍物类型索引号

            obstacle.width = ObstacleTypes[obs_typeid].width;//障碍物类型索引号对应的障碍物的尺寸
            obstacle.length = ObstacleTypes[obs_typeid].length;

            // printf("type %d %d %f \n ", obs_typeid, radar_front->type[i], theta);

            // center position of obstacle
            obstacle.x = RADARFRONT_X + radar_front->ranges[i]*cos(theta) + obstacle.length/2.0;//障碍物中心点在车辆坐标系中的位置
            obstacle.y = RADARFRONT_Y - radar_front->ranges[i]*sin(theta)- copysign(obstacle.width/2.0, theta);
            radarobstacles.Obstacles.push_back(obstacle);
        }
    }
    return;
}

// TODO: CHECK THE coor
void RadarBack_Callback( const prescan::VehradarConstPtr& radar_back ){
    for (int i = 0; i < radar_back->ranges.size(); i++ ){//遍历后方雷达传感器检测到的所有障碍物
        perception::RadarObstacle obstacle;
        if ( radar_back->ranges[i] > 0 ){
            double theta = deg2rad(radar_back->theta[i]);
            // index - 1
            int obs_typeid = radar_back->type[i] - 1;

            obstacle.width = ObstacleTypes[obs_typeid].width;
            obstacle.length = ObstacleTypes[obs_typeid].length;

            obstacle.x = RADARBACK_X - radar_back->ranges[i]*cos(theta);
            obstacle.y = RADARBACK_Y + radar_back->ranges[i]*sin(theta);

            radarobstacles.Obstacles.push_back(obstacle);

        }
    }
    return;
}

void RadarLeft_Callback( const prescan::VehradarConstPtr& radar_left ){
    for (int i = 0; i < radar_left->ranges.size(); i++ ){//遍历左方雷达传感器检测到的所有障碍物
        perception::RadarObstacle obstacle;
        if ( radar_left->ranges[i] > 0 ){
            double theta = deg2rad( radar_left->theta[i] );
            // index - 1
            int obs_typeid = radar_left->type[i] - 1;

            obstacle.width = ObstacleTypes[obs_typeid].width;
            obstacle.length = ObstacleTypes[obs_typeid].length;

            obstacle.x = RADARLEFT_X + radar_left->ranges[i]*sin(theta);
            obstacle.y = RADARLEFT_Y + radar_left->ranges[i]*cos(theta);

            radarobstacles.Obstacles.push_back(obstacle);

        }
    }
    return;
}

void RadarRigth_Callback( const prescan::VehradarConstPtr& radar_right ){
    for (int i = 0; i < radar_right->ranges.size(); i++ ){//遍历右方雷达传感器检测到的所有障碍物
        perception::RadarObstacle obstacle;
        if ( radar_right->ranges[i] > 0 ){
            double theta = deg2rad( radar_right->theta[i] );
            // index - 1
            int obs_typeid = radar_right->type[i] - 1;

            obstacle.width = ObstacleTypes[obs_typeid].width;
            obstacle.length = ObstacleTypes[obs_typeid].length;

            obstacle.x = RADARLEFT_X - radar_right->ranges[i]*sin(theta);
            obstacle.y = RADARLEFT_Y - radar_right->ranges[i]*cos(theta);

            radarobstacles.Obstacles.push_back(obstacle);

        }
    }
    return;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
    
    ros::Publisher radar_pub          = nh.advertise<perception::RadarObstacles> ("RadarObstacles", 1);/*1 为消息队列，
    因为旧的消息没有用，所以不需要将队列设置大于1*/

    ros::Subscriber radar_front_sub     = nh.subscribe("/radar_front", 1, RadarFront_Callback);
    // ros::Subscriber radar_back_sub      = nh.subscribe("/radar_back", 1, RadarBack_Callback);
    // ros::Subscriber radar_left_sub      = nh.subscribe("/radar_left", 1, RadarLeft_Callback);
    // ros::Subscriber radar_right_sub     = nh.subscribe("/radar_right", 1, RadarRigth_Callback);
    /*从这个地方可以看出，此代码只整合了radar_front（前方雷达所扫描到的障碍物，即前方的障碍物）*/

    setobstalce();

    while (ros::ok)
    {
        radarobstacles.header.stamp = ros::Time::now();
        radar_pub.publish(radarobstacles);
        /*
        if (!radarobstacles.Obstacles.empty()) { 
            for (int i = 0; i < radarobstacles.Obstacles.size(); i++){
                printf("%f %f %f %f\n", radarobstacles.Obstacles[i].x, radarobstacles.Obstacles[i].y, radarobstacles.Obstacles[i].width, radarobstacles.Obstacles[i].length);
            }
            break; 
        }
        */
        radarobstacles.Obstacles.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/*此程序的大概思路:整合车辆周围的障碍物信息，并将其以消息的形式发布出来--zrs*/
