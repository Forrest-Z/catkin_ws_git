#include "define.h"
class TrajEvaluator
{
public:
    TrajEvaluator(path path_, vector<Obstale> obstales_, double lane_center_d){
        lon_objective_cost = LonObjectiveCost(path_); //纵向行驶距离代价
        lon_comfort_cost   = LonComfortCost(path_);   //纵向舒适度代价
        lon_collision_cost = LonCollisionCost(path_, obstales_);//碰撞代价

        lat_offset_cost    = LatOffsetCost(path_, lane_center_d);//横向偏移量代价
        lat_comfort_cost   = LatComfortCost(path_);          //横向舒适度代价

    };

    double TrajCost(){

        printf("lon_objective_cost, lon_comfort_cost, lon_collision_cost, lat_offset_cost,lat_comfort_cost %f %f %f %f %f \n",
                lon_objective_cost, lon_comfort_cost, lon_collision_cost, lat_offset_cost,lat_comfort_cost);  
 
        traj_cost = lon_objective_cost * FLAGS_weight_lon_objective +
                    lon_comfort_cost * FLAGS_weight_lon_jerk + 
                    lon_collision_cost * FLAGS_weight_lon_collision + 
                    lat_offset_cost * FLAGS_weight_lat_offset+
                    lat_comfort_cost * FLAGS_weight_lat_comfort + 
                    plan_time_cost * FLAGS_weight_plan_time + 
                    addition_lanechange_cost * FLAGS_weight_lanechange;
       //总代价为各个分代价乘以各自权重求和
        return traj_cost;
        
    }
    // virtual ~TrajEvaluator();



private:

    double traj_cost = 0.0;
    double lon_objective_cost, lon_comfort_cost, lon_collision_cost, lat_offset_cost, 
            lat_comfort_cost, plan_time_cost, addition_lanechange_cost;

    double FLAGS_longitudinal_jerk_upper_bound = 4.0;
    double FLAGS_weight_lon_objective = 10.0;
    double FLAGS_weight_dist_travelled = 10.0;
    double FLAGS_weight_lon_jerk = 1.0;
    double FLAGS_weight_lon_collision = 7.0;
    double FLAGS_weight_centripetal_acceleration = 1.5;
    double FLAGS_weight_lat_offset = 1.0;
    double FLAGS_weight_lat_comfort = 5.0;
    double FLAGS_weight_plan_time = 30;   // by jyw
    double FLAGS_weight_lanechange = 2.0; // by jyw
    double FLAGS_lat_offset_bound = 0.5;
    double FLAGS_weight_opposite_side_offset = 3.0;
    double FLAGS_weight_same_side_offset = 1.0;
    double FLAGS_lattice_epsilon = 1e-6;
    double FLAGS_lon_collision_yield_buffer = 3.0;
    double FLAGS_lon_collision_overtake_buffer = 5.0;
    
	double LonObjectiveCost(path& path_){
		double distance = path_.s.back() - path_.s.front();
		return (1.0/(1.0+distance))*FLAGS_weight_dist_travelled;
	}

    double LonComfortCost(path path_){
        // same as apollo
        double cost_sqr_sum = 0.0;
        double cost_abs_sum = 0.0;
        for (int i = 0; i < path_.s.size(); i++)
        {
            double jerk = path_.s_ddd[i];
            double cost = jerk / FLAGS_longitudinal_jerk_upper_bound;
            cost_sqr_sum += cost * cost;
            cost_abs_sum += std::fabs(cost);
        }
        return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
    }

    double LonCollisionCost(path path_, vector<Obstale> obstales_)
    {
        double cost = 0;
        for (int i = 0; i < path_.s.size(); i++){
            cost += LonCollisionCost(path_.s[i], path_.l[i], path_.t[i], obstales_);
        }
        return cost;
    }

    double LonCollisionCost(double s, double d, double t, vector<Obstale> obstales_){
        double cost_sqr_sum = 0;
        double cost_abs_sum = 0;
        for (Obstale obs : obstales_){
        if (fabs(obs.d - d) < 2.5){
            double currob_s = obs.s + obs.speed * t;
            //  1,2 veh's length
            double obs_low_bound = currob_s - 0.5*obs.length;
            double obs_up_bound  = currob_s + 0.5*obs.length;
            double cost = 0;

            if (s < obs_low_bound){
            double dist = obs_low_bound - FLAGS_lon_collision_yield_buffer - s;
            cost = std::exp(-2*dist * dist);
            }
            else if (s > obs_up_bound){
            double dist = obs_up_bound - FLAGS_lon_collision_overtake_buffer - s;
            cost = std::exp(-2*dist * dist); 
            }

            cost_sqr_sum += cost * cost;
            cost_abs_sum += fabs(cost);
        }
        }
        return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
    }
    
	double LatOffsetCost(path& path_, double lane_center_d){
		double lat_offset_cost_sqr_sum, lat_offset_cost_abs_sum;
		for (auto l_ : path_.l){
      // lat cost same as apollo
      double lat_offset_cost = (l_-lane_center_d) / FLAGS_lat_offset_bound;
      if (path_.l.front()*l_ < 0.0) {
        lat_offset_cost_sqr_sum += lat_offset_cost * lat_offset_cost * FLAGS_weight_opposite_side_offset;
        lat_offset_cost_abs_sum += std::fabs(lat_offset_cost) * FLAGS_weight_opposite_side_offset;
      } else {
        lat_offset_cost_sqr_sum += lat_offset_cost * lat_offset_cost * FLAGS_weight_same_side_offset;
        lat_offset_cost_abs_sum += std::fabs(lat_offset_cost) * FLAGS_weight_same_side_offset;
      }
		}
		return lat_offset_cost_sqr_sum / (lat_offset_cost_abs_sum+FLAGS_lattice_epsilon);
	}
	double LatComfortCost(path& path_){
        // almost same as apollo
		double max_cost = 0.0;
		for (int i = 0; i < path_.l.size(); i++){
			double cost = path_.l_dd[i]*pow(path_.s_d[i], 2) + path_.l_d[i]*path_.s_dd[i];
			max_cost = std::max(max_cost, std::fabs(cost));
		}
		return max_cost;		
	}

    /*
    double AdditionLaneChange(){
        if ( (-1 == get_mylane(l0)) || (-1 == get_mylane(l_)) ){ 
			printf("l0, l_ l1 heading %f %f %f \n", l0, l_, l1);
            addition_lanechange_cost += 1000;
        }
        else { 
            addition_lanechange_cost += fabs( get_mylane(l0) - get_mylane(l_) ); 
        }
    }
    */
};