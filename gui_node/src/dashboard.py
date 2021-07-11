#! /usr/bin/env python
# -*- coding:utf-8 -*-
# 显示preview 和　cmd 的速度以及角度

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from veh_msgs.msg import Cmd, Preview, Sts

def draw_scale(c_x, c_y, r, number = 10, minimum = 0, maximum = 100):
	for i in range(number+1):
		f_x = c_x + r * np.cos(np.pi - i * np.pi / (number))
		f_y = c_y + r * np.sin(np.pi - i * np.pi / (number))

		s_x = c_x + (r-0.2) * np.cos(np.pi - i * np.pi / (number))
		s_y = c_y + (r-0.2) * np.sin(np.pi - i * np.pi / (number))		

		plt.plot([s_x, f_x], [s_y, f_y], color='g')

		plt.text(s = str(int(minimum+i*(maximum-minimum)/number)), x = s_x-0.1, y=s_y-0.1, fontsize=12)

def draw_circle(c_x, c_y, r, color='r',title="velocity"):
	x,y = [], []
	for i in range(0,46):
		theta = float(8*i) / 180 * np.pi
		x.append(c_x + r * np.cos(theta))
		y.append(c_y + r * np.sin(theta))

		plt.plot(x,y,color=color)
		plt.title(title)
		plt.axis('off')

def needle(x,y,r,angle):
	x1 = x + r * np.cos(np.pi - angle)
	y1 = y + r * np.sin(np.pi - angle)
	
	return [x, x1], [y, y1]
		
class Dashboard_Test:
	def __init__(self):
		sts_sub = rospy.Subscriber("DispSts", Sts, self.callback)

		self.max_v = 40.0; self.max_steer_angle = 50.0
		self.center_x = 2; self.center_y = 2; self.r = 2
		self.target_v, self.target_s_a, self.current_v, self.current_s_a = 0,np.pi/2,0,np.pi/2
		
		fig = plt.figure(figsize=(6,6))
		ax = plt.axes(xlim=(0,6),ylim=(0,6))
	
		plt.subplot(221)
		draw_circle(self.center_x, self.center_y, self.r, title="current_velocity")
		draw_scale(self.center_x,self.center_y,self.r,number=10,maximum=self.max_v)
		self.line_c_v, = plt.plot([], [],marker="o")

		plt.subplot(222)
		draw_circle(self.center_x, self.center_y, self.r,color='b',title="current_steer_angle")
		draw_scale(self.center_x, self.center_y, self.r, number=10, minimum=self.max_steer_angle,maximum=-self.max_steer_angle)
		self.line_c_s, = plt.plot([], [],marker="o")

		plt.subplot(223)
		draw_circle(self.center_x, self.center_y, self.r, title="dest_velocity")
		draw_scale(self.center_x,self.center_y,self.r,number=10,maximum=self.max_v)
		self.line_t_v, = plt.plot([], [],marker="o")

		plt.subplot(224)
		draw_circle(self.center_x, self.center_y, self.r,color='b',title="dest_steer_angle")
		draw_scale(self.center_x, self.center_y, self.r, number=10, minimum=self.max_steer_angle, maximum=-self.max_steer_angle)
		self.line_t_s, = plt.plot([], [],marker="o")	
		
		timer = rospy.Timer(rospy.Duration(1.0 / 10.0), self.timer_callback)
		animation = FuncAnimation(fig, self.timer_callback, interval=1)
		plt.axis('off')
		plt.show()


	def callback(self, message):
		# if message.header.frame_id != "Invalid frame ID":
		if message.DestSpd < 100:
			self.target_v = message.DestSpd / self.max_v * np.pi
			self.target_s_a = (-message.DestStrAngle/16.5) / self.max_steer_angle * np.pi/2 + np.pi/2
			self.current_v = message.CurSpd / self.max_v * np.pi
			self.current_s_a = (-message.CurStrAngle/16.5)/ self.max_steer_angle * np.pi/2 + np.pi/2
		else:
			self.target_v = 0
			self.target_s_a = np.pi/2
			self.current_v = 0
			self.current_s_a = np.pi/2					
		
	def timer_callback(self, event):
		self.line_c_v.set_data(needle(self.center_x, self.center_y, self.r, self.current_v))
		self.line_c_s.set_data(needle(self.center_x, self.center_y, self.r, self.current_s_a))

		self.line_t_v.set_data(needle(self.center_x, self.center_y, self.r, self.target_v))
		self.line_t_s.set_data(needle(self.center_x, self.center_y, self.r, self.target_s_a))

		return self.line_c_v, self.line_c_s, self.line_t_v, self.line_t_s	
	
if __name__ == '__main__':
	rospy.init_node('test_view')
	Dashboard_Test()
	rospy.spin()
	timer.shutdown()