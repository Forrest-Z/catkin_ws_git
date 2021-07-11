std_msgs/Header header
float32 x            ##车辆后轴中心当前的全局坐标x，后轴中心
float32 y            ##车辆后轴中心当前的全局坐标y
float32 heading      ##车辆当前的航向角，与Y轴正方向逆时针夹角
float32 spd          ##车辆当前的速度
float32 acc          ##车辆当前的加速度
float32 s            ##车辆当前的s_l坐标s
float32 ds           ##车辆当前的s_l坐标ds
float32 dds          ##车辆当前的s_l坐标dds
float32 d            ##车辆当前的s_l坐标l
float32 dl           ##车辆当前的s_l坐标dl
float32 ddl          ##车辆当前的s_l坐标ddl
int8    lane         ##车辆当前 所在车道
float32 time_on_lane ##车辆在同一车道上的时间