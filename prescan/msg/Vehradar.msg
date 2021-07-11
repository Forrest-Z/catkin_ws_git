std_msgs/Header header
float64[20] ranges  ##障碍物与当前车辆的距离,包含20个元素的数组
int8[20] type       ##障碍物类型
float64[20] theta   ##障碍物所在的方位角 
float64[20] dopplervel  ##障碍物的多普勒速度