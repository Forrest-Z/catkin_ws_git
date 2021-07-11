#include <iostream>
#include "ros/ros.h"
#include <cmath>
#include "../include/pid/pidcontrol.h"

void PIDController::Init(const PidConf &pid_conf)
{
    error_sum_throttle=0;
    k_p_throttle=pid_conf.k_p_throttle;
    k_i_throttle=pid_conf.k_i_throttle;
    k_d_throttle=pid_conf.k_d_throttle;

    error_sum_pressure=0;
    k_p_pressure=pid_conf.k_p_pressure;
    k_i_pressure=pid_conf.k_i_pressure;
    k_d_pressure=pid_conf.k_d_pressure;

    error_sum_pressure_up=0;
    k_p_pressure_up=pid_conf.k_p_pressure_up;
    k_i_pressure_up=pid_conf.k_i_pressure_up;
    k_d_pressure_up=pid_conf.k_d_pressure_up;

    error_sum_pressure_low=0;
    k_p_pressure_low=pid_conf.k_p_pressure_low;
    k_i_pressure_low=pid_conf.k_i_pressure_low;
    k_d_pressure_low=pid_conf.k_d_pressure_low;
}

//high speed
historic_msg_throttle PIDController::Control_throttle(float error,float v_r)
{

    historic_msg_throttle msg_throttle;

    msg_throttle.throttle=k_p_throttle*error+k_i_throttle*error_sum_throttle+k_d_throttle;
    error_sum_throttle=error_sum_throttle+error;

    return msg_throttle;
}

historic_msg_pressure PIDController::Control_pressure(float error,float v_r)
{

    historic_msg_pressure msg_pressure;

    msg_pressure.pressure=k_p_pressure*error+k_i_pressure*error_sum_pressure+k_d_pressure;
    error_sum_pressure=error_sum_pressure+error;

    return msg_pressure;
}


//lower speed
historic_msg_pressure PIDController::Control_pressure_up(float error,float v_r)
{
    int i=0;
    historic_msg_pressure msg_pressure_up;
    //table for k_d:
    float v_r_up[]={0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};
    float k_d_up[]={18.2,18.1,18.0,17.8,17.7,17.4,16.5,15.5,14.0,13.0};

    for(i=0;i<=9;i++){
            if(v_r>=v_r_up[i]&&v_r<=v_r_up[i+1]){
                k_d_pressure_up=k_d_up[i]+(v_r-v_r_up[i])*(k_d_up[i+1]-k_d_up[i])/(v_r_up[i+1]-v_r_up[i]);
                ROS_INFO("k_d_up: %f",k_d_pressure_up);
                break;
            }
    }
    msg_pressure_up.pressure_up=-1*(k_p_pressure_up*error+k_i_pressure_up*error_sum_pressure_up)+k_d_pressure_up;
    
    if(v_r<0.1){
        msg_pressure_up.pressure_up=73;
    }
    error_sum_pressure_up=error_sum_pressure_up+error;

    return msg_pressure_up;
}

historic_msg_pressure PIDController::Control_pressure_low(float error,float v_r)
{
    int i;
    historic_msg_pressure msg_pressure_low;
    float v_r_down[]={0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9};
    float k_d_down[]={7.6,8.5,9.0,9.3,9.5,10.5,10.0,9.5,8.5};//the value of k_d_pressure_low reflect to the value of v_r from 0.1 to 0.9
    
    for(i=0;i<=9;i++){
        if(v_r>=v_r_down[i]&&v_r<=v_r_down[i+1]){
                k_d_pressure_low=k_d_down[i]+(v_r-v_r_down[i])*(k_d_down[i+1]-k_d_down[i])/(v_r_down[i+1]-v_r_down[i]);
                ROS_INFO("k_d_down: %f",k_d_pressure_low);
                break;
        }
    }
    msg_pressure_low.pressure_low=-1*(k_p_pressure_low*error+k_i_pressure_low*error_sum_pressure_low)+k_d_pressure_low;
    
    if(v_r<0.1){
        msg_pressure_low.pressure_low=73;
    }
    error_sum_pressure_low=error_sum_pressure_low+error;

    return msg_pressure_low;
}//***PID_postion_type