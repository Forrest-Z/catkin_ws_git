// time: 2019.3.31
// author: jiangyuewei
// optimal traj on frenet frame

#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

#include "helpers.h"
#include "traj_evaluator.h"
#include "polynomial.h"
#include "bounding_box.h"
#include "frenet_to_cartesian.h"


using namespace std;

bool DEBUG = true;

struct obstacle
{
	double x;
	double y;
};

int get_mylane(double veh_d){
	//LANEWIDTH在define.h文件中定义，为3.5
  if ( ((-0.5*LANEWIDTH+0.5*myveh.width)<veh_d) && (veh_d<(0.5*LANEWIDTH)) ) { return 0; }
  else if ( (0.5*LANEWIDTH<=veh_d) && (veh_d<=1.5*LANEWIDTH-0.5*myveh.width) ) { return 1; }
  else
    printf("mistake occured with no lane %f \n", veh_d);
  return -1;
}


constexpr double km_h2m_s(double x)
{
    return x/3.6;
}


class Frenet_Optimal_Plan
{
public:
//Frenet_Optimal_Plan()为有参构造函数
	Frenet_Optimal_Plan(double s0, double curr_speed, double curr_acc, 
			double l0, double dl0, double ddl0, vector<Obstale> obstales, int lane, double runningtime, float veh_heading)
	{

		obstales_ = obstales; 
		veh_heading_ = veh_heading;
    get_center_d(lane, lane_center_d);      //lane代表几车道，lane_center_d代表所有车道对应的宽度，获取lane_center_d
    generate_d_condition(lane, d_condition, runningtime);   //获取d_condition
		// if (DEBUG) { printf("d_condition size %d \n", d_condition.size()); }
		all_pathes = frenet_pathes(s0, curr_speed, curr_acc, l0, dl0, ddl0);
		sort(all_pathes.begin(), all_pathes.end(), path_comp);//path_comp为排序规则

	};
	// ~Frenet_Optimal_Plan();

	path optimal_path(){
		// cout << reference_line.size() << "	reference_line	" << endl;
		for (int i = 0; i<all_pathes.size(); i++){
			if (IsPassBoundryCheck(all_pathes[i])) { return all_pathes[i]; }
		}
    sort(all_pathes.begin(), all_pathes.end(), path_comp_collision);//path_comp_collision为排序规则		
		// check again to debug by jyw
		if ( InCollision( all_pathes.front() )) { 
			printf(" output a collision path with max TTC s d %f %f %f \n", all_pathes.front().collision_time,all_pathes.front().s.back(), all_pathes.front().l.back());
			for (int i = 0; i < obstales_current.size(); i++){
				printf(" obstacle x y %f %f \n", obstales_current[i].center_x(), obstales_current[i].center_y());
			}
		}
		printf("############################### \n");
		return all_pathes.front();
	}

private:
	pathes all_pathes;
	pathes all_pathes_frenet, all_pathes_cartesian, collision_free_pathes;
  vector<Obstale> obstales_;
	vector<Box2d> obstales_current;
	vector<double> d_condition = {lane_center_d-sample_offset_d, lane_center_d+0.0, lane_center_d+sample_offset_d};
	vector<double> s_condition = {8, 12, 25};
  int lane_center_d;
	float veh_heading_;

  void get_speed_range(vector<double> & speed_range, double curr_speed, double plan_time){
    speed_range.clear();
    // double speed_min = max(0.0,   curr_speed - (myveh.max_deceleration)*plan_time);
    // double speed_max = min(ref_v, curr_speed + (0.5*myveh.max_acceleration)*plan_time);
		// NO ACC
		double speed_min = curr_speed;
		double speed_max = curr_speed+2.5;
		
    double delta_speed = 2;
    for (double speed = speed_min; speed <= speed_max;)
		{   
      speed_range.emplace_back(speed);
      speed += delta_speed;
    }
		//speed_range={curr_speed, curr_speed+2, curr_speed+4}
    return;

  }

  void get_current_obstacle(vector<Obstale> obstales, int index, vector<Box2d> & obstales_current){
    obstales_current.clear();
    for (auto obs : obstales){
      double x_ = obs.x + obs.vx * index*delta_t;
      double y_ = obs.y + obs.vy * index*delta_t;
      Box2d ob_box({x_, y_}, obs.theta, obs.length, obs.width);
      obstales_current.emplace_back(ob_box);  
			// printf("obstacle x y min_x max_x min_y max_y theta %f %f %f %f %f %f %f \n", 
			//				x_, y_, ob_box.min_x(), ob_box.max_x(), ob_box.min_y(), ob_box.max_y(), obs.theta);
    }
  }


  void get_center_d(int lane, int & lane_center_d)
	{
    if (0 == lane)
      lane_center_d = 0;
    else if (1 == lane)      //一车道
      lane_center_d = LANEWIDTH;
    else if (2 == lane)      //两车道
      lane_center_d = 2*LANEWIDTH;
    else
      cout << " error without lane_center_d";
    return;    
  }

  void generate_d_condition(int lane, vector<double> & d_condition, double runningtime)
	{
		d_condition = {0, LANEWIDTH}; 
		return;		
		if ( -1 == lane ) 
		{ 
			d_condition = {0, LANEWIDTH}; 
			return;
		}
    if (runningtime < 1.0){
      // traffic rule 
			//sample_offset_d在define.h中定义，为0.5
      if (0 == lane)
        d_condition = {0-sample_offset_d, 0+0.0, 0+sample_offset_d};
				//如果车在0车道，d_condition={-0.5， 0, 0.5}
      else if (1 == lane)
        d_condition = {LANEWIDTH-sample_offset_d, LANEWIDTH+0.0, LANEWIDTH+sample_offset_d};
				//如果车在1车道，d_condition={3， 3.5, 4}
      else if (2 == lane)
        d_condition = {2*LANEWIDTH-sample_offset_d, 2*LANEWIDTH+0.0, 2*LANEWIDTH+sample_offset_d};
				//如果车在2车道，d_condition={6.5， 7, 7.5}
      else{
					cout << " error without d_condition";
					d_condition = {0-sample_offset_d, 0+0.0, 0+sample_offset_d};
			}
      return;      
    }

		d_condition = {0-sample_offset_d, 0+0.0, 0+sample_offset_d, LANEWIDTH-sample_offset_d, LANEWIDTH+0.0, LANEWIDTH+sample_offset_d};
    return;
  }

	static bool path_comp(const path& a, const path& b){
    return a.cost < b.cost;
	}//升序排列

  static bool path_comp_collision(const path& a, const path& b){
    return a.collision_time > b.collision_time;
  }//降序排列

	pathes frenet_pathes(double s0, double curr_speed, double curr_acc, double l0, double dl0, double ddl0)
	{
		pathes all_pathes_;
		for (double l1 : d_condition) //从数组d_condition中依次取出元素赋值给double型变量l1
		{      //d_condition = {lane_center_d-sample_offset_d, lane_center_d+0.0, lane_center_d+sample_offset_d}
			for (double s : s_condition)  //从数组s_condition中依次取出元素赋值给double型变量s
			{    //s_condition = {8, 12, 25}
				// lat
				//  横向状态的采样，末状态采样到达l1需要的s值，根据六个已知条件得到的为一个五次多项式，l=l(s)
				Quintic_Polynomial quintic_polynomial(l0, dl0, ddl0, l1, 0, 0, s);//得到五次多项式对应的6个系数
				//得到横向轨迹的采样后，在与纵向轨迹采样结果两两结合
				for (double plan_time = plan_time_flag; plan_time <= plan_time_flag; plan_time++)
				{     //此处好像有错误，plan_time应从1-8,此处应将第一个表达式改为：plan_time=1  --zrs
          std::vector<double> speed_range;
          get_speed_range(speed_range, curr_speed, plan_time); //获取speed_range数组,为speed_range={curr_speed, curr_speed+2, curr_speed+4}
          for (double ref_v_ : speed_range)	//向量speed_range中依次取出元素赋值给double型变量ref_v_
					{
						// TODO: quattic_polynomial should feed s, ds, dds
						// follow and overtake（跟车或超车），跟车或超车的末状态加速度为0，此代码没有考虑巡航状态，s0为0：因为每一时刻的s坐标系0点在车辆所处位置
  					Quartic_Polynomial quartic_polynomial(0, curr_speed, curr_acc, ref_v_, 0, plan_time);//得到4次多项式对应的5个系数
  					// v = v_obstacle; s = ?

  					// combine
  					double s_, ds_, dds_, ddds_, l_, dl_, ddl_, dddl_;
  					path tem_path;
  					// double delta_speed = 0;
  					for (double t = 0; t<plan_time+0.1*delta_t;)
						{
  						s_ = quartic_polynomial.Evaluate(0, t);   //四次多项式的0阶导数
  						ds_ = quartic_polynomial.Evaluate(1, t);  //四次多项式的1阶导数
  						dds_ = quartic_polynomial.Evaluate(2, t);
  						ddds_ = quartic_polynomial.Evaluate(3, t);
  						tem_path.t.push_back(t);
  						tem_path.s.push_back(s_+s0);
  						tem_path.s_d.push_back(ds_);
  						tem_path.s_dd.push_back(dds_);
  						tem_path.s_ddd.push_back(ddds_);
  						std::array<double, 3> s_conditions = {s_+s0, ds_, dds_};

  						if (s_ >= s)
  							s_ = s;
  						l_ = quintic_polynomial.Evaluate(0, s_);   //五次多项式的0阶导数
  						dl_ = quintic_polynomial.Evaluate(1, s_);
  						ddl_ = quintic_polynomial.Evaluate(2, s_);
  						dddl_ = quintic_polynomial.Evaluate(3, s_);
  						tem_path.l.push_back(l_);
  						tem_path.l_d.push_back(dl_);
  						tem_path.l_dd.push_back(ddl_);
  						tem_path.l_ddd.push_back(dddl_);

  						// trans to cartesian;
  					  std::array<double, 3> d_conditions = {l_, dl_, ddl_};
  					  PathPoint matched_ref_point = MatchToPath(reference_line, tem_path.s.back());//此处的reference_line定义于define.h中,赋值于lattice_planner.cpp
																								  																				 //故此处的reference_line有值
  					  double x = 0.0;
              double y = 0.0;
              double theta = 0.0;
              double kappa = 0.0;
              double v = 0.0;
              double a = 0.0;

              const double rs = matched_ref_point.s;
              const double rx = matched_ref_point.x;
              const double ry = matched_ref_point.y;
              const double rtheta = matched_ref_point.theta;
              const double rkappa = matched_ref_point.kappa;
              const double rdkappa = matched_ref_point.dkappa;
            //（rs,rx,ry,rtheta,rkappa,rdkappa）为s_对应参考线上点的相应信息
  						frenet_to_cartesian( rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, 
  							&x, &y, &theta, &kappa, &v, &a);
           //依据（rs,rx,ry,rtheta,rkappa,rdkappa）和（s_+s0, ds_, dds_）、（l_, dl_, ddl_），获取车辆在frenet坐标系
					 //下的各种信息，然后转换到迪卡尔坐标系下，得到迪卡尔坐标系下的（x,y,theta,kappa,v,a）
  						tem_path.theta.push_back(theta);
  						tem_path.kappa.push_back(kappa);
  						tem_path.v.push_back(v);
  						tem_path.a.push_back(a);
  						tem_path.x.push_back(x);
  						tem_path.y.push_back(y);


  						t += delta_t;

  					}

						TrajEvaluator traj_evaluator(tem_path, obstales_, lane_center_d);
						tem_path.cost = traj_evaluator.TrajCost();

						printf("cost length sample_s sample_d %f %f %f %f \n", 
										tem_path.cost, tem_path.s.back()-tem_path.s.front(), s, l1);

  					all_pathes_.push_back(tem_path);
          }
				}
			}
		}
		return all_pathes_;
	};

	bool IsPassBoundryCheck(path& path_)
	{
		// speed check
		for (double speed : path_.s_d){
			if (speed > MAX_SPEED){
				if (DEBUG) { printf("speed limit\n"); }
				return false;
			}
		}
		// accel check
		for (double acc : path_.s_dd){
			if ((acc > myveh.max_acceleration) || (acc < myveh.max_deceleration)){
				if (DEBUG) { printf("acc limit \n"); }
				return false;
			}
		}
    // jerk check
    for (double jerk : path_.s_ddd){
      if (fabs(jerk) > MAX_JERK){
				if (DEBUG) { printf("jeck limit \n"); }
        return false;
      }
    }    
		// curvature check
		for (double kappa : path_.kappa){
			if (fabs(kappa) > MAX_CURVATURE){
				if (DEBUG) { printf("curvature limit \n"); }
				return false;
			}
		}

    // check out of the lane
    for (double l_ : path_.l){
      if ((l_ < -0.5*LANEWIDTH+0.5*myveh.width) || (l_ > 1.5*LANEWIDTH-0.5*myveh.width)){
        if (DEBUG) { printf(" out of lane %f \n", l_); }
        return false;
      }
    }

		// collision check
		if (InCollision(path_)){  
        if (DEBUG) { printf("path in collision s d %f %f \n", path_.s.back(), path_.l.back()); }
        return false;
      }

		return true;

	};


	bool InCollision(path & discretized_trajectory) 
	{
    // cout << "start check collision " << endl;
    // the coor we plan on the center of the rear axis
	  double ego_width = myveh.width;
	  double ego_length = myveh.length;
	  double back_edge_to_center = myveh.back_edge_to_center;

	  for (int i = 0; i < discretized_trajectory.x.size(); ++i) {
	    double ego_theta = discretized_trajectory.theta[i];
	    Box2d ego_box(
	        {discretized_trajectory.x[i], discretized_trajectory.y[i]}, ego_theta, ego_length, ego_width);

	    double shift_distance = ego_length / 2.0 - back_edge_to_center;

	    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
	                    shift_distance * std::sin(ego_theta)};
      // trans to the real center of vehicle
	    ego_box.Shift( shift_vec );
      get_current_obstacle(obstales_, i, obstales_current);

	    for (const auto& obstacle_box : obstales_current) {
	      if (ego_box.HasOverlap(obstacle_box)) {
          discretized_trajectory.collision_time = i*delta_t;
	        return true;
	      }
	    }

	  }
	  return false;
	}
	
};

