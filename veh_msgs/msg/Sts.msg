std_msgs/Header header
float32 DestSpd               ## 目标速度，单位 km/h    
float32 DestStrAngle          ## 目标角度，单位 °
float32 DestAcc               ## 目标加速度 单位 g
float32 DestStrAngleSpd       ## 目标角速度 单位 °/s
float32 CurSpd                ## 当前GPS速度，单位 km/h
float32 CurAcc                ## 当前GPS加速度 单位g
float32 CurStrAngle           ## 当前角度，单位 °
int8  CurGear               ## 当前档位
float32 CurFWSpd              ## 当前车辆前轴速度，单位 km/h
int8  CurMode               ## 当前车辆运行模式：MANUAL，AUTO，REMOTE
int8  CurBrkSts             ## 当前车辆制动状态
