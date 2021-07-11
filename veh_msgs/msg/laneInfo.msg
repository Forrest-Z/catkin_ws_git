#LaneInfo.msg
bool isLeftLane  
bool isRightLane
bool isOwnLane
lanePoint[] lane0   #valid if isLeftLane = true
lanePoint[] lane3   #valid if isRightLane = true
lanePoint[] lane1   #valid if isOwnLane = true
lanePoint[] lane2   #valid if isOwnLane = true

#lane type define 
# 1: dotted lane type
# 2: solide lane type
int8 lane0Type 
int8 lane1Type
int8 lane2Type
int8 lane3Type

 

