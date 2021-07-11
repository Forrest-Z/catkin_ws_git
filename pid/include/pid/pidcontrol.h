struct PidConf {
  float k_p_throttle;
  float k_i_throttle;
  float k_d_throttle;
  float k_p_pressure;
  float k_i_pressure;
  float k_d_pressure;

  float k_p_pressure_up;
  float k_i_pressure_up;
  float k_d_pressure_up;
  float k_p_pressure_low;
  float k_i_pressure_low;
  float k_d_pressure_low;
};

struct historic_msg_throttle
{
	float throttle;
};
struct historic_msg_pressure
{
	float pressure;
  float pressure_low;
  float pressure_up;
};
class PIDController {
 public:
  void Init(const PidConf &pid_conf);//此函数在此头文件中声明，在pid_control.cpp中定义
                                     //***set the value of v_o and throttle as 0 when there is no any control
                                     //***set the value of k_p,k_i,k_d


  historic_msg_throttle Control_throttle(float error,float v_r);/*Control_throttle(float error,float v_r)是属于historic_msg_throttle
  这个数据结构的，Control_throttle(float error,float v_r)此函数的返回值为msg_throttle--zrs
                                                            **get the control message of throttle */

  historic_msg_pressure Control_pressure(float error,float v_r);//***get the control message of pressure

  historic_msg_pressure Control_pressure_up(float error,float v_r);//***get the control message of throttle under lower speed of 1.089m/s

  historic_msg_pressure Control_pressure_low(float error,float v_r);//***get the control message of pressure under lower speed of 1.089m/s

 protected:
  float k_p_throttle;
  float k_i_throttle;
  float k_d_throttle;
  float error_sum_throttle;

  float k_p_pressure;
  float k_i_pressure;
  float k_d_pressure;
  float error_sum_pressure;

  float k_p_pressure_up;
  float k_i_pressure_up;
  float k_d_pressure_up;
  float error_sum_pressure_up;

  float k_p_pressure_low;
  float k_i_pressure_low;
  float k_d_pressure_low;
  float error_sum_pressure_low;
};
