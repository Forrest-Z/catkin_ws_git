#include <vector>
struct Veh_State{
  double x;
  double y;
  double s;
  double d;
  double dl;
  double ddl;
  double heading;
  double spd;
  double acc;
  int lane;
  double time_on_lane;
};
 
typedef struct Obstale {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double speed;
  double theta = 0;
  double width = 2;
  double length = 5;
} Obstale;

struct mkz_veh
{
  /// mkz param
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;
  double length = 4.933;
  double width = 2.11;
  double height = 1.48;
  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.0;
  double max_deceleration = -6.0;
  double max_steer_angle = 8.20304748437;
  double max_steer_angle_rate = 6.98131700798;
  double min_steer_angle_rate = 0;
  double steer_ratio = 16;
  double wheel_base = 2.8448;
  double wheel_rolling_radius = 0.335;
  double max_abs_speed_when_stopped = 0.2;
};

// NOTE: params of Audi_A8 is not ture, need to rewrite
struct Audi_A8{
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;
  double length = 4.933;
  double width = 2.11;
  double height = 1.48;
  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.0;
  double max_deceleration = -6.0;
  double max_steer_angle = 8.20304748437;
  double max_steer_angle_rate = 6.98131700798;
  double min_steer_angle_rate = 0;
  double steer_ratio = 16;
  double wheel_base = 2.8448;
  double wheel_rolling_radius = 0.335;
  double max_abs_speed_when_stopped = 0.2;  
};

struct Truck{
  double front_edge_to_center = 3.89;
  double back_edge_to_center = 1.043;
  double left_edge_to_center = 1.055;
  double right_edge_to_center = 1.055;
  double length = 4.933;
  double width = 2.11;
  double height = 1.48;
  double min_turn_radius = 5.05386147161;
  double max_acceleration = 2.0;
  double max_deceleration = -6.0;
  double max_steer_angle = 8.20304748437;
  double max_steer_angle_rate = 6.98131700798;
  double min_steer_angle_rate = 0;
  double steer_ratio = 24.4;
  double wheel_base = 4.8;
  double wheel_rolling_radius = 0.335;
  double max_abs_speed_when_stopped = 0.2;  
};

struct Vec2d
{
  double x;
	double y;
};

struct PathPoint
{
	double x;
	double y;
	double s;
	double theta;//轨迹点斜率
	double kappa;//轨迹点曲率
	double dkappa;//曲率的变化率
	double ddkappa;
};

struct path
{
	std::vector<double> t;
	std::vector<double> s;
	std::vector<double> s_d;
	std::vector<double> s_dd;
	std::vector<double> s_ddd;
	std::vector<double> l;
	std::vector<double> l_d;
	std::vector<double> l_dd;
	std::vector<double> l_ddd;

	std::vector<double> theta;
	std::vector<double> kappa;

	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> yaw;
	std::vector<double> ds;
	std::vector<double> curv;
	std::vector<double> v;
	std::vector<double> a;

  double cost;

  double collision_time = 10;  // if all path has collision, select path with long time
};

typedef std::vector<path> pathes;

std::vector<PathPoint> reference_line;

Truck myveh;

// if less than MIN_CollisionTime, need emergency brake
#define MIN_CollisionTime 2.0

// for traj generator 
#define LANEWIDTH 3.5
#define LOW_SPEED 3.0


#define sample_offset_d 0.50

#define plan_time_flag 8.0
#define ref_v 40.0
#define delta_t 0.1
#define kMathEpsilon 0.01


#define MAX_SPEED 40.0
#define MAX_JERK 10.0 //  m/s^3
#define MAX_CURVATURE 1.0