#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, //************************
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);//(x,y)与（map_x,map_y）两点之间的距离
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns  the closest waypoint
//当车辆航向角与 车辆所在点与参考轨迹上最近点连线与x轴的夹角 之间的差值大于90度，则输出最近点的下一个点
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, //**********************
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);//找出地图上距离车辆当前位置最近的点的序列号

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];//地图上距离车辆最近点的位置信息

  double heading = atan2((map_y-y),(map_x-x));//当前距离车辆最近点的地图点连线与X轴的夹角（-180～180度）

  double angle = fabs(theta-heading);//车辆航向与车辆位置点和最近点连线的夹角，fabs函数是一个求绝对值的函数,
  angle = std::min(2*pi() - angle, angle);
  // printf("angle heading x y map_x map_y %f %f %f %f %f %f \n", angle, heading,x, y,map_x, map_y);
  //小于等于90度，即取该点，否则取下一个点
  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;//防止没有参考线轨迹点
    }
  }
  // printf("angle %f \n", angle);
  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, //**********************************
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) 
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    // prev_wp  = maps_x.size()-1;
    prev_wp = 0;    // jyw doing this it might not loop
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y+1e-8);//加入一修正系数
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;//得到映射点的坐标，映射的目的在于判断车辆在参考线的左右位置，以及确定车辆当前的位置

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  //判断该点是在线的左端还是右端，左端为正，右端为负
  /*double alfa1=atan2(proj_x-maps_x[prev_wp],proj_y-maps_y[prev_wp]);
  double alfa2=atan2(x - maps_x[prev_wp],y - maps_y[prev_wp]);
  if(alfa1>=0){
    if(alfa2<0){alfa2=2*pi()+alfa2;}
    if ((alfa2-alfa1)>pi()&&(alfa2-alfa1)<0 {
      frenet_d *= -1;
    }
  }
  else{
    if ((alfa2-alfa1)>pi()&&(alfa2-alfa1)<0 {
      frenet_d *= -1;
  }*///by hrq***********************************************************************加入

  // if (centerToPos <= centerToRef) {
  // modify by jyw
  if (centerToPos > centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);  //s的值为从初始点到prev_wp点中间所有点两两折线的长度和
  }
  // printf("prev_wp %d %d \n", prev_wp, next_wp);

  frenet_s += distance(0,0,proj_x,proj_y);   //s的最终值还要加上从prev_wp点到映射点的直线距离

  return {frenet_s,frenet_d};//返回此时车辆在局部frenet坐标下的s和d值：坐标为参考线起始点
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


void vehframe2cartesian(float veh_x, float veh_y, float theta, float& cartesian_x, float& cartesian_y){
  float alpha = atan2(veh_y, veh_x);
  float length = sqrt(pow(veh_x,2)+ pow(veh_y,2));

  cartesian_x = -1*length*sin(alpha+theta);
  cartesian_y = length*cos(alpha+theta);
}

void trans2vehframe(float utm_x, float utm_y, float theta, float& vehframe_x, float& vehframe_y){
    float beta = atan2(utm_y, utm_x);
    float length = sqrt(pow(utm_x,2) + pow(utm_y,2));

    vehframe_x = length*cos(beta-theta-deg2rad(90.0));
    vehframe_y = length*sin(beta-theta-deg2rad(90.0));//只有旋转，没有平移
    return;    
}

void cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa, const double x, const double y,
    const double v, const double a, const double theta, const double kappa,
    std::array<double, 3>* const ptr_s_condition,
    std::array<double, 3>* const ptr_d_condition) {
  const double dx = x - rx;
  const double dy = y - ry;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  ptr_d_condition->at(0) =
      std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
  // printf("dx, dy, rtheta, cross_rd_nd, ptr_d_condition->at(0) %f %f %f %f %f \n", dx, dy, rtheta, cross_rd_nd, ptr_d_condition->at(0));
  const double delta_theta = theta - rtheta;
  const double tan_delta_theta = std::tan(delta_theta);
  const double cos_delta_theta = std::cos(delta_theta);

  const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
  ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

  const double kappa_r_d_prime =
      rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

  ptr_d_condition->at(2) =
      -kappa_r_d_prime * tan_delta_theta +
      one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
          (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

  ptr_s_condition->at(0) = rs;

  ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

  const double delta_theta_prime =
      one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
  ptr_s_condition->at(2) =
      (a * cos_delta_theta -
       ptr_s_condition->at(1) * ptr_s_condition->at(1) *
           (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
      one_minus_kappa_r_d;
  return;
}



#endif  // HELPERS_H