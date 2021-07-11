// unit: speed: m/s; acc: m/s^2; jeck: m/s^3

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include "math.h"
#include "algorithm"
#include <map>
#include <vector>
#include <complex>
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"

#include "control_manual/Cmd.h"
#include "lattice_planner/path.h"
#include "lattice_planner/define.h"
#include "lattice_planner/helpers.h"
#include "prescan/Vehstate.h"

using namespace std_msgs;
using namespace std;

control_manual::Cmd   planner_cmd;
path optimal_path;
float DestSpd_1;

void VehStsCallback(const prescan::VehstatePtr& sts)
{
  planner_cmd.GPSSpd          = sts->spd;
  planner_cmd.GPSAcc          = sts->acc;
  return;
  
}
void PreviewCmdCallback(const lattice_planner::pathPtr& opt_path){
  if(!opt_path->path.empty()){
    optimal_path.x.clear();
    optimal_path.y.clear();
    optimal_path.v.clear();
    optimal_path.collision_time=10;

    for(int i=0;i<opt_path->path.size();i++){
      optimal_path.x.push_back(opt_path->path[i].x);
      optimal_path.y.push_back(opt_path->path[i].y);
      optimal_path.v.push_back(opt_path->path[i].v);
    }

    optimal_path.collision_time=opt_path->collision_time;
  }
    return;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "preview");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);

  ros::Subscriber vehstate_sub         = nh.subscribe("veh_loc_frenet", 1,  VehStsCallback);
  ros::Subscriber opt_path_sub       = nh.subscribe("opt_path", 1,  PreviewCmdCallback);

  ros::Publisher  planner_cmd_pub      = nh.advertise<control_manual::Cmd> ("/planner_cmd", 1, true);

  planner_cmd.DestSpd             = 0;
  planner_cmd.DestStrAngle        = 0;
  planner_cmd.DestAcc             = 0;
  planner_cmd.DestStrAngleSpd     = 0;
  planner_cmd.DestBrkPos          = 0;
  planner_cmd.DestAccPos          = 0;
  planner_cmd.DestGear            = 1;
  planner_cmd.EmgBrkFlag          = 0;


  while (ros::ok)
  {
      //ROS_INFO("%lu",optimal_path.x.size());
      if (optimal_path.collision_time > MIN_CollisionTime&&optimal_path.x.size()>15){//判断：若规划出来的路径的碰撞时间是否大于最小碰撞时间，若是，则继续前进，否则紧急停车
        // spd; heading
        // preview 
        //车辆坐标系下，利用模型预测控制算法，计算方向盘角度
        float PreviewPoint_x = optimal_path.x[15];
        float PreviewPoint_y = optimal_path.y[15];//取规划出来路基的第16个点作为预瞄点(注意：该坐标系中，车辆前进方向为x方向，车辆左侧方向为y方向)

        float CO = (pow(PreviewPoint_y, 2)+pow(PreviewPoint_x, 2)) / (2*PreviewPoint_y+kMathEpsilon);//计算得到车辆转弯半径（注意：可利用三角形相似原理计算得到，其值可正可负）
        // CO must lager than min_turn_radius
        CO = std::copysign(fmax(fabs(CO), myveh.min_turn_radius), CO);//限制车辆转弯半径在车辆允许范围内
        float deststrangle = rad2deg(std::atan(myveh.wheel_base/CO) * myveh.steer_ratio);//根据车辆转弯半径计算得到对应车辆方向盘转角（注意：向右为正，向左为负）
        planner_cmd.DestStrAngle = std::copysign(std::min( 500.0f, fabs(deststrangle)), deststrangle);//限制最大方向盘转角为500度
        if(planner_cmd.DestSpd<=1.089&&(planner_cmd.DestStrAngle>=400||planner_cmd.DestStrAngle<=-400))
	      planner_cmd.DestSpd=1.1;//最低转弯速度为1.1，停车时把方向盘回位且小于400度，否则车可能不能停下来
        DestSpd_1=planner_cmd.DestSpd;//***by hrq

        planner_cmd.DestSpd      = optimal_path.v[10];

        if(planner_cmd.DestSpd>=0.05&&planner_cmd.DestSpd<=0.1)//***by hrq
          planner_cmd.DestSpd=0.1;
        else if(planner_cmd.DestSpd<0.05)
         planner_cmd.DestSpd=0;

        if(DestSpd_1!=planner_cmd.DestSpd)
          planner_cmd.DestSpd_1= DestSpd_1;//***by hrq,get the previous velocity,may not be used
        
        planner_cmd.EmgBrkFlag   = 0;
      }
      else{
        // emergency brake
        planner_cmd.DestSpd = 0;
        planner_cmd.EmgBrkFlag   = 1;
        //ROS_INFO("collision time %f", optimal_path.collision_time);
      }

    planner_cmd_pub.publish(planner_cmd);

    ros::spinOnce();
    loop_rate.sleep();
    // break;
    // if ( !planner_path.poses.empty() ) { break; }
  }
  return 0;
}