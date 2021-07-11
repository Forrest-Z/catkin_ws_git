#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "algorithm"
#include <map>
#include <vector>
#include <complex>
#include <iostream>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include "prescan/Vehstate.h"//编译msg消息生成的头文件
#include "prescan/Vehradar.h"//编译msg消息生成的头文件
#include "../include/prescan/HelperPrescan.h"
#include "../include/prescan/tcp.h"

#define PORT 8087
//#define SERVER_IP "169.254.149.241"
#define SERVER_IP "169.254.19.55"
#define BUFF_SIZE 1024
#define FILE_NAME_MAX_SIZE 512  
#define CAMERA_SIZE 640*480*3
// #define CAMERA_SIZE 1024*20
#define LIDAR_SIZE 7680
#define LIDAR_RANGE 240
#define LIDAR_BEAMS 32
#define LIDAR_DISTANCE 150
// Lidar location
#define LIDAR_X 2.0
#define LIDAR_Y 0.0
#define LIDAR_Z 1.32
// lidar deg
#define LIDAR_parallel 120.0
#define LIDAR_vertical 16.0

#define RADAR_MAX_OBJECT 20   // max objects can be detected, the number should be same as the Vehradar.msg
#define RADAR_SIZE RADAR_MAX_OBJECT*4*4         // 4 means there are 4 radars
#define TCP_BUFF_SIZE CAMERA_SIZE+(5+LIDAR_SIZE)*sizeof(float)+RADAR_SIZE*sizeof(double) + 4

#define gps_offset_x 1.77

void lidarscan2point(); 

float lidar_vertical_resolution;
float lidar_paraller_resolution;

ros::Publisher state_pub, image_pub, lidar_pub, lidar_point_pub;
ros::Publisher radar_front_pub, radar_back_pub, radar_left_pub, radar_right_pub;

prescan::Vehstate vehstate;
sensor_msgs::Image image;
sensor_msgs::LaserScan lidar;
sensor_msgs::PointCloud lidar_point;
prescan::Vehradar radar_front, radar_back, radar_left, radar_right;
int sockfd;
int c_addr_len, readlen;
struct sockaddr_in c_addr;
//***the function take the angular massage from deg to rad
float deg2rad(float x) { return (x*M_PI/180.0); }

//***the function take the GPS location to the Rear axis center
void GPS2RearAxisCenter(float gps_x, float gps_y, float heading, float& x, float& y){
    x = gps_offset_x*sin(heading) + gps_x;//gps_offset_x为车辆坐标位置点与车辆后轴的纵向距离，值为1.77（查看prescan中的“dynamics”获得）
    y = gps_y - gps_offset_x*cos(heading);
    return;
}

int feed_sensorframe(){

    image.data.assign(CAMERA_SIZE, 0);
    lidar.ranges.assign(LIDAR_SIZE, 0);

    c_addr_len=sizeof(struct sockaddr_in);
    
    sockfd=socket(AF_INET,SOCK_STREAM,0);
    if(-1==sockfd)
    {
        ROS_WARN("creat socket error");
        return -1;
    }

    memset(&c_addr,0,c_addr_len);
    c_addr.sin_family = AF_INET;
    c_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    c_addr.sin_port = htons(PORT);

    char buffer[BUFF_SIZE];
    int opt = 1;
    setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));
    if(connect(sockfd,(struct sockaddr*)&c_addr,c_addr_len)==-1)
    {
        ROS_WARN("socket connect error");
        return -1;
    }

    memset(buffer, 0, BUFF_SIZE);

    if (send(sockfd,buffer,BUFF_SIZE,0)==-1) 
    {
        ROS_WARN("send error");
        return -1;
    }
    unsigned char Buff[BUFF_SIZE];
    int receive_cout = 0;
    unsigned char tcp_buff[TCP_BUFF_SIZE];
    int tcp_buff_point = 0;
    float state_buff[5];
    double radar_buff[RADAR_SIZE];
    float lidar_buff[LIDAR_SIZE];
    while (1){                                  
        readlen = read(sockfd, Buff, BUFF_SIZE);//read（）函数：从Buff中读写BUFF_SIZE个bytes到sockfd中，返回the number written，or -1
        if (readlen <= 0) break;
        memcpy(&tcp_buff[tcp_buff_point], &Buff, readlen);//从Buff中复制readlen个数据到tcp_buff中
        tcp_buff_point += readlen;
    }
    close(sockfd);
    // feed to msg
    memcpy(&image.data[0],      &tcp_buff, CAMERA_SIZE);//消息image的来源,memcpy()函数的作用：从tcp_buff中复制CAMERA_SIZE个bytes到image.data[0]中
    memcpy(&state_buff[0],      &tcp_buff[CAMERA_SIZE], sizeof(state_buff));
    memcpy(&lidar.ranges[0],    &tcp_buff[CAMERA_SIZE+sizeof(state_buff)], sizeof(float)*LIDAR_SIZE);//消息lidar的来源
    ////  这里有额外的四个字节，float　和　double　都在结构体时，double前有额外的四个字节
    ////　上述均为测试获得，暂未找到实际支持说明    －－姜跃为
    memcpy(&radar_buff[0],      &tcp_buff[CAMERA_SIZE+sizeof(state_buff)+sizeof(float)*LIDAR_SIZE+4], sizeof(radar_buff));
//**以上四个memcpy函数作用：将tcp传输过来的数据转存到image.data[0]、state_buff[0]、lidar.ranges[0]、radar_buff[0]四个数组中--zrs
    //printf("radar_buff %f %f %d \n", radar_buff[0], lidar.ranges[1], tcp_buff_point);//-hrq
    float gps_x, gps_y, x, y;
    gps_x               = state_buff[0];
    gps_y               = state_buff[1];
    vehstate.heading    = 2*M_PI - deg2rad(state_buff[2]);//输入的航向角为与Y轴顺时针的夹角，此操作将其转换为相对Y轴逆时针夹角
    vehstate.spd        = state_buff[3];
    vehstate.acc        = state_buff[4];
    //将state_buff数组中的数据转到Vehstate.msg中的heading、spd、acc--zrs
    GPS2RearAxisCenter(gps_x, gps_y,vehstate.heading, x, y);//将车辆位置转换到车辆后轴中心
    vehstate.x          = x;
    vehstate.y          = y;
    //将GPS2RearAxisCenter（）函数求出的x、y转到Vehstate.msg中的x、y--zrs
    
    for (int i = 0; i<RADAR_MAX_OBJECT; i++){
        radar_front.ranges[i]       = radar_buff[i];
        radar_back.ranges[i]        = radar_buff[i+RADAR_MAX_OBJECT];
        radar_left.ranges[i]        = radar_buff[i+RADAR_MAX_OBJECT*2];
        radar_right.ranges[i]       = radar_buff[i+RADAR_MAX_OBJECT*3];

        radar_front.type[i]         = radar_buff[i+RADAR_MAX_OBJECT*4];
        radar_back.type[i]          = radar_buff[i+RADAR_MAX_OBJECT*5];
        radar_left.type[i]          = radar_buff[i+RADAR_MAX_OBJECT*6];
        radar_right.type[i]         = radar_buff[i+RADAR_MAX_OBJECT*7]; 

        radar_front.theta[i]        = radar_buff[i+RADAR_MAX_OBJECT*8];
        radar_back.theta[i]         = radar_buff[i+RADAR_MAX_OBJECT*9];
        radar_left.theta[i]         = radar_buff[i+RADAR_MAX_OBJECT*10];
        radar_right.theta[i]        = radar_buff[i+RADAR_MAX_OBJECT*11];

        radar_front.dopplervel[i]    = radar_buff[i+RADAR_MAX_OBJECT*12];
        radar_back.dopplervel[i]     = radar_buff[i+RADAR_MAX_OBJECT*13];
        radar_left.dopplervel[i]     = radar_buff[i+RADAR_MAX_OBJECT*14];
        radar_right.dopplervel[i]    = radar_buff[i+RADAR_MAX_OBJECT*15];//四个消息radar_front back left right 的来源--zrs
    }    

    lidarscan2point();
    return 0;

}

void lidarscan2point(){
    geometry_msgs::Point32 lidar_point_;
    float range, theta, alpha;
    float length,utm_beta,utm_alpha;
    lidar_point.points.clear();
    for (int i = 0; i < LIDAR_BEAMS; i++){
        for (int j = 0; j < LIDAR_RANGE; j++){
            range = lidar.ranges[i*LIDAR_RANGE+j];
            if (range < LIDAR_DISTANCE){
                // top to down
                theta = deg2rad(lidar_vertical_resolution * 0.5*(LIDAR_BEAMS - i));
                // left to right
                alpha = deg2rad(lidar_paraller_resolution * (j-0.5*LIDAR_RANGE));
                // on vehframe
                lidar_point_.z = range * sin(theta);
                lidar_point_.y = -1*range * cos(theta) * sin(alpha);
                lidar_point_.x = range * cos(theta) * cos(alpha) + LIDAR_X;//注意，此时计算得到的激光雷达点的坐标为原点在车的后轴中心点，但坐标轴方向与车辆坐标轴一致
                /*
                // set coor_center_to_gps
                // lidar_point_.x -= gps_offset_x; (ignore)
                // trans2UTM
                length = sqrt(pow(lidar_point_.x,2)+pow(lidar_point_.y,2));
                utm_beta = std::atan2(lidar_point_.y, lidar_point_.x);
                utm_alpha = -1*(utm_beta+vehstate.heading);
                // printf("utm_beta, utm_alpha, lidar_point_.y %f %f %f \n", utm_beta, utm_alpha, lidar_point_.y);
                lidar_point_.x = length*sin(utm_alpha); // +vehstate.x;
                lidar_point_.y = length*cos(utm_alpha); //+vehstate.y;
                */
                float cartesian_x, cartesian_y;
                vehframe2cartesian(lidar_point_.x, lidar_point_.y, vehstate.heading, cartesian_x, cartesian_y);//将车辆坐标轴下的点的坐标转换至原点仍然在车辆后轴中心，但坐标轴为全局坐标轴
                lidar_point_.x = cartesian_x;
                lidar_point_.y = cartesian_y;
                // add to lidar_point.points
                lidar_point.points.emplace_back(lidar_point_);//消息lidar_point（激光雷达位置点）的来源
            }

        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensorframe");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    state_pub = nh.advertise<prescan::Vehstate> ("vehstate", 1, true);
    //创建一个发布者，名为：state_pub，发布名为vehstate的topic，消息类型为：prescan::Vehstate--zrs
    image_pub = nh.advertise<sensor_msgs::Image> ("image", 1, true);
    lidar_pub = nh.advertise<sensor_msgs::LaserScan> ("lidar", 1, true);
    lidar_point_pub = nh.advertise<sensor_msgs::PointCloud> ("lidar_point", 1, true);
    radar_front_pub = nh.advertise<prescan::Vehradar> ("radar_front", 1, true);
    radar_back_pub = nh.advertise<prescan::Vehradar> ("radar_back", 1, true);
    radar_left_pub = nh.advertise<prescan::Vehradar> ("radar_left", 1, true);
    radar_right_pub = nh.advertise<prescan::Vehradar> ("radar_right", 1, true);

    lidar_vertical_resolution = LIDAR_vertical / LIDAR_BEAMS;  //resolution-分辨率
    lidar_paraller_resolution = LIDAR_parallel / LIDAR_RANGE;

    image.height = 480;//prescan中图像的高
    image.width = 640;//宽
    image.is_bigendian = 0;
    image.encoding = "rgb8";//图像的格式为rgb
    image.step = 1920;//图像的通道数为3，所以640*3=1920

    lidar.angle_min = -60;
    lidar.angle_max = 60;
    lidar.angle_increment = 0.5;//角度的增量为0.5度
    lidar.header.frame_id = "lidar_front";

    lidar_point.header.frame_id = "UTM_Coordination";

    while (ros::ok)
    {
        vehstate.header.stamp = ros::Time::now();
        image.header.stamp = ros::Time::now();
        lidar.header.stamp = ros::Time::now();
        lidar_point.header.stamp = ros::Time::now();
        radar_front.header.stamp = ros::Time::now();
        radar_back.header.stamp = ros::Time::now();
        radar_left.header.stamp = ros::Time::now();
        radar_right.header.stamp = ros::Time::now();

        if (-1 == feed_sensorframe()){
            ROS_WARN("sensorframe tcp init failed !");
        }
        state_pub.publish(vehstate);
        image_pub.publish(image);
        lidar_pub.publish(lidar);
        lidar_point_pub.publish(lidar_point);
        radar_front_pub.publish(radar_front);
        radar_back_pub.publish(radar_back);
        radar_left_pub.publish(radar_left);
        radar_right_pub.publish(radar_right);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/*此程序将TCP传输过来的prescan中的 { 1.车辆位置信息（坐标、航向角）、运动信息（速度、加速度），2.以及四个毫米波雷达
（前后左右的radar）获取的障碍物的相关信息（类型、距离、速度和方位角），3.一个激光雷达所处的位置（x、y、z信息）
和雷达的检测范围 } 发布出来。*/
/*发布出vehstate image lidar lidar_point radar_front radar_back radar_left radar_right 这些消息*/
