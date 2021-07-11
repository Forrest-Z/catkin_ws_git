1.function:
this ros package is the longitudinal control for the vehcle use the method of PID.

lon_control.cpp:the main code for the method of PID,subscribe the message of the dest_velocity and the real_velocity,get the control commend "throttle" and "presure",then publish them,"velocyty_pid_pub" is for the rqt_plot,"throttle_pid_pub" is for the longitudinal control.

2.usage:
(1)type: "cd catkin_ws"
(2)type: "catkin_make"
(3)type: "roscore"
(4)type: "roslaunch pid pid.launch"
(5)type: "rosrun rqt_plot rqt_plot" -> type the sign of "+"
