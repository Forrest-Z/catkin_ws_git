/*
    sub the global_path
    smooth the path ---> reference line
    NOTE: refline should include x,y,heading,kappa,dkappa
*/

//  map ego veh and obstacles to Frenet Frame

#include "ros/ros.h"
#include "nav_msgs/Path.h"

#include "prescan/Vehstate.h"
#include "refline/ReferenceLine.h"
#include "perception/RadarObstacles.h"

#include "../include/refline/spline.h"
#include "../include/refline/HelperRefline.h"

//#include "trajectory_generator.h"


nav_msgs::Path referenceline_test;//显示参考线，便于可视化
refline::ReferenceLine referenceline;//参考线轨迹点信息
prescan::Vehstate veh_loc_frenet;//frenet坐标系下的车辆状态信息
perception::RadarObstacles RadarObstaclesFrenet;//frenet坐标系下障碍物的状态信息

std::vector<double> map_x, map_y;
const double delta_s = 0.3;//参考线采样间隔

void RadarCallback(const perception::RadarObstaclesPtr& radarobs){//订阅得到障碍物在车辆迪卡尔坐标系下的状态信息
                                                                 //将其放入RadarObstaclesFrenet中
    RadarObstaclesFrenet.Obstacles = radarobs->Obstacles;
    return;
}


void LocCallback(const prescan::VehstateConstPtr& loc){//订阅得到车辆在世界迪卡尔坐标系下的状态信息
                                                       //将其放入veh_loc_frenet中
    veh_loc_frenet.x = loc->x;
    veh_loc_frenet.y = loc->y;
    veh_loc_frenet.heading = loc->heading;
    veh_loc_frenet.spd = loc->spd;
    veh_loc_frenet.acc = loc->acc;
    return;
}

void GlobalPathCallback(const nav_msgs::PathConstPtr& path) //47-310行
{//订阅获取得到的全局路径信息，对其进行平滑化，然后再离散化
    if (!path->poses.empty()) 
    { 
        double theta = veh_loc_frenet.heading+deg2rad(90.0);//将车辆航向角转换成与x轴正方向的夹角

        referenceline_test.poses.clear();
        referenceline.ReferencePoints.clear();
        tk::spline x_given_s;
        tk::spline y_given_s;
        float accumulate_s = 0;
        std::vector<double> x_, y_, s_;

//改：map只保存了一次地图信息，以至于再次获取得到全局地图后规划无效
        if ( map_x.empty() ){

            for (int i = 0; i+1 < path->poses.size(); i++){
                map_x.push_back(path->poses[i].pose.position.x);
                map_y.push_back(path->poses[i].pose.position.y);                
            }
        }
        else if(path->poses.size()>1){
            if(map_x[map_x.size()-1]!=path->poses[path->poses.size()-2].pose.position.x){
            map_x.clear();
            map_y.clear();
            for (int i = 0; i+1 < path->poses.size(); i++){
                map_x.push_back(path->poses[i].pose.position.x);
                map_y.push_back(path->poses[i].pose.position.y);                
            }
            }
        }//将全局路径信息放入map中



/*
        if ( map_x.empty() ){//map只保存了一次地图信息，以至于再次获取得到全局地图后规划无效
            for (int i = 0; i+1 < path->poses.size(); i++){
                map_x.push_back(path->poses[i].pose.position.x);
                map_y.push_back(path->poses[i].pose.position.y);                
            }
        }//将全局地图信息放入map中
*/

        int nextwaypoint = NextWaypoint(veh_loc_frenet.x, veh_loc_frenet.y,theta, map_x, map_y);//地图匹配，车辆在全局坐标系下的x,y,航向角（X轴逆时针），地图点X，Y
        int index;
        if (nextwaypoint == 0) { index = nextwaypoint; }
        else { index = nextwaypoint-1; }
        int i = index;
        while (i < path->poses.size() - 1){//以距离车辆当前点最近的全局路径点为起始点计算出全局路径后面各点的S值，并将这些点的x，y，以及计算出的s存入x_,y_,s_中
            x_.push_back(path->poses[i].pose.position.x);
            y_.push_back(path->poses[i].pose.position.y);
            if (i != index)
                accumulate_s += sqrt(pow(path->poses[i].pose.position.x-path->poses[i-1].pose.position.x, 2) + pow(path->poses[i].pose.position.y-path->poses[i-1].pose.position.y, 2));
                
            s_.push_back(accumulate_s);
            if (accumulate_s > 150) {break;}
            i++;
        }//将从距离车辆最近点的地图路径上长度150米内的所有路径点的x,y（UTM坐标）和s装入x_,y_,s_中
        // float trans_theta;
        // trans_theta = std::atan2(path->poses[index+1].pose.position.y-path->poses[index].pose.position.y,
        //                        path->poses[index+1].pose.position.x-path->poses[index].pose.position.x);

        // x_given_s.set_boundary();
        x_given_s.set_points(s_, x_);//三次样条插值拟合道路中心线的s和x,得到x(s)
        y_given_s.set_points(s_, y_);//三次样条插值拟合道路中心线的s和y,得到y(s)

        for(i=0;i<x_.size();i++)
        {
            ROS_INFO("now:%f %f %f",s_[i],x_[i],y_[i]);
        }

        accumulate_s = 0;
        geometry_msgs::PoseStamped tem;
        refline::ReferencePoint refpoint;

        while (accumulate_s < s_.back())
        {//最后一个点没有取到
            // check the ref line in rviz
            // need std msg     
            float cartesian_x = x_given_s(accumulate_s) - veh_loc_frenet.x;//得到参考线路点转换至原点在车辆位置点，但坐标轴为全局坐标系坐标轴
            float cartesian_y = y_given_s(accumulate_s) - veh_loc_frenet.y;

            tem.pose.position.x = cartesian_x;
            tem.pose.position.y = cartesian_y;
            referenceline_test.poses.push_back(tem);

            refpoint.x = cartesian_x;
            refpoint.y = cartesian_y;
            refpoint.s = accumulate_s;

            if (referenceline.ReferencePoints.empty()) { refpoint.theta = 0; }
            else { refpoint.theta = 
            std::atan2(refpoint.y - referenceline.ReferencePoints.back().y, 
                        refpoint.x - referenceline.ReferencePoints.back().x); }//参考线上第i个参考点处的倾斜角，第一个点的为0
            
            // might not right
            // TODO: check kappa and dkappa
            //refpoint.kappa = 0;
            //refpoint.dkappa = 0;//by jyw

            referenceline.ReferencePoints.push_back(refpoint);

            accumulate_s += delta_s;//平滑曲线离散化，每隔0.3米取一个点
        }
        //计算曲率和曲率变化率,by hrq
        for(int j1=0;j1<referenceline.ReferencePoints.size();j1++){

            if(j1==0||(j1<=(referenceline.ReferencePoints.size()-1)&&j1>=(referenceline.ReferencePoints.size()-2))){
                refpoint.kappa=0;
                refpoint.dkappa=0;
            }//第一个点和最后两个点的曲率和曲率变化率为0
            else{
                refpoint.kappa=(referenceline.ReferencePoints[j1].theta-referenceline.ReferencePoints[j1+1].theta)/delta_s;
                refpoint.dkappa=(referenceline.ReferencePoints[j1].theta-2*referenceline.ReferencePoints[j1+1].theta+referenceline.ReferencePoints[j1+2].theta)/(delta_s*delta_s);
            }
            referenceline.ReferencePoints[j1].kappa=refpoint.kappa;
            referenceline.ReferencePoints[j1].dkappa=refpoint.dkappa;
        }


        // feed vehloc on frenet frame
        /*
        void cartesian_to_frenet(
        const double rs, const double rx, const double ry, const double rtheta,
        const double rkappa, const double rdkappa, const double x, const double y,
        const double v, const double a, const double theta, const double kappa,
        std::array<double, 3>* const ptr_s_condition,
        std::array<double, 3>* const ptr_d_condition)
        */

        //改：将映射基准线由粗糙路径转换为光滑后的路径
        std::vector<double> x_splin,y_splin,s_splin;
        for (int j2=0;j2< referenceline.ReferencePoints.size();j2++)
        {//将各个路点的X,Y,以及对应的S分别存入x_splin,y_splin,s_splin
            x_splin.push_back(x_given_s(referenceline.ReferencePoints[j2].s));
            y_splin.push_back(y_given_s(referenceline.ReferencePoints[j2].s));
            s_splin.push_back(referenceline.ReferencePoints[j2].s);
        }
        //车辆当前位置映射到frenet坐标系上的映射点的s和d值 s_d={frenet_s,frenet_d}
        std::vector<double> s_d = getFrenet(veh_loc_frenet.x, veh_loc_frenet.y, theta,x_splin, y_splin);//获取车辆的
         double rs = s_d[0];//车辆当前位置映射到frenet坐标系上的映射点的s值
        double rx = x_given_s(rs);//s=rs时,道路中心线对应位置点的x值
        double ry = y_given_s(rs);//s=rs时,道路中心线对应位置点的y值
        double rx_next = x_given_s(rs+delta_s);
        double ry_nexy = y_given_s(rs+delta_s);
        double rtheta = std::atan2(ry_nexy-ry, rx_next-rx);

        std::array<double, 3> s_condition = {0,0,0};
        std::array<double, 3> d_condition = {0,0,0};

        cartesian_to_frenet(rs, rx, ry, rtheta, 0, 0, veh_loc_frenet.x, veh_loc_frenet.y,
                            veh_loc_frenet.spd, veh_loc_frenet.acc, theta, 0,&s_condition, &d_condition);

        veh_loc_frenet.s = s_condition[0];
        veh_loc_frenet.ds = s_condition[1];
        veh_loc_frenet.dds = s_condition[2];
        veh_loc_frenet.d = d_condition[0];
        veh_loc_frenet.dl = d_condition[1];
        veh_loc_frenet.ddl = d_condition[2];
        // ROS_INFO("get frenet s d rx ry x y theta %f %f %f %f %f %f %f", 
        // s_d[0], veh_loc_frenet.d, rx, ry, veh_loc_frenet.x, veh_loc_frenet.y, theta);

        float RadarObstaclesCartesian_x, RadarObstaclesCartesian_y;
        for (int i = 0; i<RadarObstaclesFrenet.Obstacles.size(); i++){       
            vehframe2cartesian(RadarObstaclesFrenet.Obstacles[i].x, RadarObstaclesFrenet.Obstacles[i].y,
                            veh_loc_frenet.heading, RadarObstaclesCartesian_x, RadarObstaclesCartesian_y);

            std::vector<double> s_d = 
            getFrenet(RadarObstaclesCartesian_x+veh_loc_frenet.x, RadarObstaclesCartesian_y+veh_loc_frenet.y, theta, x_splin, y_splin);
            double rs = s_d[0];
            double rx = x_given_s(rs);
            double ry = y_given_s(rs);
            
            double rx_next = x_given_s(rs+delta_s);
            double ry_nexy = y_given_s(rs+delta_s);
            double rtheta = std::atan2(ry_nexy-ry, rx_next-rx);

            RadarObstaclesFrenet.Obstacles[i].s = s_d[0];
            RadarObstaclesFrenet.Obstacles[i].l = s_d[1];

            // TODO: spd might not true
            /*
            RadarObstaclesFrenet.Obstacles[i].speed /= 
                cos(2*M_PI-RadarObstaclesFrenet.Obstacles[i].theta-(veh_loc_frenet.heading-rtheta));
            */
            
            RadarObstaclesFrenet.Obstacles[i].speed = 0;
            // theta here means the obstacle heaing orientation
            // different from theta in radar node
            RadarObstaclesFrenet.Obstacles[i].theta = rtheta;    // same as the lane,might not right
            RadarObstaclesFrenet.Obstacles[i].x = RadarObstaclesCartesian_x;
            RadarObstaclesFrenet.Obstacles[i].y = RadarObstaclesCartesian_y;
/*
        std::vector<double> s_d = getFrenet(veh_loc_frenet.x, veh_loc_frenet.y, theta,x_, y_);
        //输入车辆在全局坐标下的x,y,theta(与x轴逆时针)，地图信息x_.y_
        //获取得到车辆在原粗糙路径地图坐标系下的位置，s原点为平滑参考线的起点
        //可能会致使误差较大？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？/

         ROS_INFO("s_d %f %f %f %f %f ", s_d[0], s_d[1], theta, veh_loc_frenet.x, veh_loc_frenet.y);
        double rs = s_d[0];//车辆当前位置映射到frenet坐标系上的映射点的s值
        double rx = x_given_s(rs);//映射点的全局x值
        double ry = y_given_s(rs);//映射点的全局y值
        
        double rx_next = x_given_s(rs+delta_s);
        double ry_nexy = y_given_s(rs+delta_s);
        double rtheta = std::atan2(ry_nexy-ry, rx_next-rx);

        std::array<double, 3> s_condition = {0,0,0};
        std::array<double, 3> d_condition = {0,0,0};

        cartesian_to_frenet(rs, rx, ry, rtheta, 0, 0, veh_loc_frenet.x, veh_loc_frenet.y,
                            veh_loc_frenet.spd, veh_loc_frenet.acc, theta, 0,&s_condition, &d_condition);

        veh_loc_frenet.s = s_condition[0];
        veh_loc_frenet.ds = s_condition[1];
        veh_loc_frenet.dds = s_condition[2];
        veh_loc_frenet.d = d_condition[0];
        veh_loc_frenet.dl = d_condition[1];
        veh_loc_frenet.ddl = d_condition[2];
        // ROS_INFO("get frenet s d rx ry x y theta %f %f %f %f %f %f %f", 
        // s_d[0], veh_loc_frenet.d, rx, ry, veh_loc_frenet.x, veh_loc_frenet.y, theta);

        float RadarObstaclesCartesian_x, RadarObstaclesCartesian_y;
        for (int i = 0; i<RadarObstaclesFrenet.Obstacles.size(); i++){       
            vehframe2cartesian(RadarObstaclesFrenet.Obstacles[i].x, RadarObstaclesFrenet.Obstacles[i].y,
                            veh_loc_frenet.heading, RadarObstaclesCartesian_x, RadarObstaclesCartesian_y);

            std::vector<double> s_d = 
            getFrenet(RadarObstaclesCartesian_x+veh_loc_frenet.x, RadarObstaclesCartesian_y+veh_loc_frenet.y, theta, x_, y_);
            double rs = s_d[0];
            double rx = x_given_s(rs);
            double ry = y_given_s(rs);
            
            double rx_next = x_given_s(rs+delta_s);
            double ry_nexy = y_given_s(rs+delta_s);
            double rtheta = std::atan2(ry_nexy-ry, rx_next-rx);

            RadarObstaclesFrenet.Obstacles[i].s = s_d[0];
            RadarObstaclesFrenet.Obstacles[i].l = s_d[1];

            // TODO: spd might not true
            /*
            RadarObstaclesFrenet.Obstacles[i].speed /= 
                cos(2*M_PI-RadarObstaclesFrenet.Obstacles[i].theta-(veh_loc_frenet.heading-rtheta));
            */
        /*    
            RadarObstaclesFrenet.Obstacles[i].speed = 0;
            // theta here means the obstacle heaing orientation
            // different from theta in radar node
            RadarObstaclesFrenet.Obstacles[i].theta = rtheta;    // same as the lane,might not right
            RadarObstaclesFrenet.Obstacles[i].x = RadarObstaclesCartesian_x;
            RadarObstaclesFrenet.Obstacles[i].y = RadarObstaclesCartesian_y;


            */
        }

        return;
    }

    return;
    // TODO: use 5_th polynomial to smooth


}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "refer0ence_line");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
    
    ros::Publisher refpath_test_pub     = nh.advertise<nav_msgs::Path> ("referenceline_test", 1);
    ros::Publisher refpath_pub          = nh.advertise<refline::ReferenceLine> ("referenceline", 1);
    ros::Publisher veh_loc_frenet_pub   = nh.advertise<prescan::Vehstate> ("veh_loc_frenet", 1);
    ros::Publisher radarobs_frenet_pub  = nh.advertise<perception::RadarObstacles> ("radar_obstacles_frenet", 1);

    ros::Subscriber Path_sub = nh.subscribe("/GlobalPath", 1, GlobalPathCallback);
    ros::Subscriber loc_sub  = nh.subscribe("vehstate", 1, LocCallback);
    ros::Subscriber radar_sub  = nh.subscribe("RadarObstacles", 1, RadarCallback);

    referenceline_test.header.frame_id = "UTM_Coordination";
    referenceline.header.frame_id = "UTM_Coordination";
    while (ros::ok)
    {
        referenceline_test.header.stamp = ros::Time::now();
        referenceline.header.stamp = ros::Time::now();
        veh_loc_frenet.header.stamp = ros::Time::now();
        RadarObstaclesFrenet.header.stamp = ros::Time::now();

        refpath_test_pub.publish(referenceline_test);
        refpath_pub.publish(referenceline);  
        veh_loc_frenet_pub.publish(veh_loc_frenet);
        radarobs_frenet_pub.publish(RadarObstaclesFrenet);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}