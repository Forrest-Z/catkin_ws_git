std_msgs/Header header
float32 GPSSpd            ## GPS 速度，单位km/h
float32 GPSAcc            ## GPS 加速度，单位 g
float32 DestSpd           ## 目标速度，单位km/h
float32 DestSpd_1         ## 变速前的前一次目标速度，单位km/h
float32 DestStrAngle      ## 目标角度，单位 °
float32 DestAcc           ## 目标加速度 单位g
float32 DestStrAngleSpd   ## 目标角速度，单位°/s 
float32 DestBrkPos        ## 目标制动踏板位置，0~1
float32 DestAccPos        ## 目标油门踏板位置，0~1
int8  DestGear          ## 目标档位
int8  EmgBrkFlag        ## 紧急制动标识