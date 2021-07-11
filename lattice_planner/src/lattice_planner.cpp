/*lattice算法步骤：
  1. get reference line (read from the map)
  2. get the obstacles
  3. compute some traj  
  4. choose the lowest cost
  5. check the boundry
  6. output the control value
*/

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

#include "prescan/Vehradar.h"
#include "prescan/Vehstate.h"
#include "refline/ReferenceLine.h"
#include "perception/RadarObstacles.h"
#include"lattice_planner/path.h"
#include"lattice_planner/path_point.h"

#include "../include/lattice_planner/trajectory_generator.h"

nav_msgs::Path  planner_path;//此处的Path是Path.h中定义的
lattice_planner::path opt_path;//此处的
lattice_planner::path_point opt_path_point;

using namespace std_msgs;
using namespace std;

 
std::vector<Obstale> obstacles;//名为obstacles的对象数组，其类型为Obstale

Veh_State veh_state;
PathPoint path_point;
path optimal_path;// 全局坐标系下的轨迹


double Max_time_on_lane = 5;
int mylane;
float DestSpd_1;

void generate_cmd(){
  
  return;
}
//获取参考线的信息
void ReferenceLineCallback(const refline::ReferenceLinePtr& refline){
  if ( !refline->ReferencePoints.empty() ){
    reference_line.clear();
    for (int i = 0; i < refline->ReferencePoints.size(); i++){
      path_point.s      = refline->ReferencePoints[i].s;
      path_point.x      = refline->ReferencePoints[i].x;
      path_point.y      = refline->ReferencePoints[i].y;
      path_point.theta  = refline->ReferencePoints[i].theta;
      path_point.kappa  = refline->ReferencePoints[i].kappa;
      path_point.dkappa = refline->ReferencePoints[i].dkappa;
      reference_line.push_back(path_point);//在reference_line末尾的加上此第i个参考点，以此循环将所有参考点都添加到reference_line数组中
    }
    return;
  }
  return;
}
//获取车辆状态信息
void VehStsCallback(const prescan::VehstatePtr& sts)
{
  veh_state.x                 = sts->x;//车辆后轴中心当前的全局坐标x，后轴中心
  veh_state.y                 = sts->y;
  veh_state.heading           = sts->heading;//车辆当前的航向角，与Y轴正方向逆时针夹角
  veh_state.spd               = sts->spd;//辆当前的速度
  veh_state.acc               = sts->acc;
  veh_state.s                 = sts->s;//车辆当前的s_l坐标s
  veh_state.d                 = sts->d;//车辆当前的s_l坐标l，此处应该是写错了，应该写为l，
  veh_state.dl                = sts->dl;
  veh_state.ddl               = sts->ddl;
  veh_state.lane              = sts->lane;//车辆当前 所在车道
   //此处是不是少了 veh_state.time_on_lane=sts->time_on_lane;--zrs
  if (( get_mylane(veh_state.d) == mylane ) && ( mylane != -1 )){
    veh_state.time_on_lane += 0.05;
    veh_state.time_on_lane = min(veh_state.time_on_lane, Max_time_on_lane);
  }
  else{
    veh_state.time_on_lane = 0;
    mylane = get_mylane(veh_state.d);
  }
  return;
  
}
//获取障碍物信息
void ObstacleCallback(const perception::RadarObstaclesConstPtr & obs){
  obstacles.clear();
  for (int i = 0; i < obs->Obstacles.size(); i++ ){
      Obstale obstacle;
      obstacle.theta = obs->Obstacles[i].theta;
      obstacle.x = obs->Obstacles[i].x;
      obstacle.y = obs->Obstacles[i].y;
      obstacle.s = obs->Obstacles[i].s;
      obstacle.d = obs->Obstacles[i].l;
      obstacle.width = obs->Obstacles[i].width;
      obstacle.length = obs->Obstacles[i].length;
      
      obstacles.push_back(obstacle);//在obstacles末尾的加上此第i个障碍物，以此循环将所有障碍物都添加到obstacles数组中
  }
  
  return;
}



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lattice_planner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);

  
  ros::Subscriber vehstate_sub         = nh.subscribe("veh_loc_frenet", 1,  VehStsCallback);//frenet坐标系下的车辆状态信息
  ros::Subscriber obstacle_sub         = nh.subscribe("radar_obstacles_frenet", 1,  ObstacleCallback);//frenet坐标系下障碍物的状态信息
  ros::Subscriber refenceline_sub      = nh.subscribe("referenceline", 1,  ReferenceLineCallback);//参考线轨迹点信息

  ros::Publisher  planner_path_pub     = nh.advertise<nav_msgs::Path> ("/planner_path", 1, true);
  
  ros::Publisher  opt_path_pub     = nh.advertise<lattice_planner::path> ("opt_path", 1, true);

  planner_path.header.frame_id = "UTM_Coordination";

  while (ros::ok)
  {

    if ( !reference_line.empty() )//如果reference_line不为空
    {
      //frenet_optimal_plan为Frenet_Optimal_Plan类的一个对象
      Frenet_Optimal_Plan frenet_optimal_plan(
        veh_state.s, veh_state.spd, veh_state.acc,
        veh_state.d, veh_state.dl, veh_state.ddl, 
        obstacles, veh_state.lane, veh_state.time_on_lane, veh_state.heading);

      optimal_path = frenet_optimal_plan.optimal_path();

      // feed optimal path to std path msg for rviz
      planner_path.poses.clear();

      opt_path.path.clear();
      opt_path.cost=0;
      opt_path.collision_time=10;

      geometry_msgs::PoseStamped path_point;
      // ROS_INFO(" optimal path x0  v.d %f %f", optimal_path.x[0], veh_state.d);
      for (int i = 0; i < optimal_path.x.size(); i++){
        path_point.pose.position.x = optimal_path.x[i];
        path_point.pose.position.y = optimal_path.y[i];  
        planner_path.poses.push_back(path_point);

        float veh_frame_x, veh_frame_y;
        trans2vehframe(optimal_path.x[i],optimal_path.y[i], veh_state.heading, veh_frame_x, veh_frame_y);
        optimal_path.x[i] = veh_frame_x;
        optimal_path.y[i] = veh_frame_y;
        // for preview 
        opt_path_point.x=optimal_path.x[i];
        opt_path_point.y=optimal_path.y[i];
        opt_path_point.v=optimal_path.v[i];
        opt_path_point.a=optimal_path.a[i];
        opt_path_point.s=optimal_path.s[i];
        opt_path_point.l=optimal_path.l[i];
        opt_path_point.t=optimal_path.t[i];
        opt_path_point.theta=optimal_path.theta[i];
        opt_path_point.kappa=optimal_path.kappa[i];
        opt_path.path.push_back(opt_path_point);
      }
      opt_path.cost=optimal_path.cost;
      opt_path.collision_time=optimal_path.collision_time;
    }

    planner_path_pub.publish(planner_path);

    opt_path_pub.publish(opt_path);

    ros::spinOnce();
    loop_rate.sleep();
    // break;
    // if ( !planner_path.poses.empty() ) { break; }
  }
  return 0;
}