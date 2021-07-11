#! /usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import message_filters
from matplotlib.animation import FuncAnimation

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from veh_msgs.msg import Cmd, Preview


class grid_map:
	def __init__(self):
		self.row_length = rospy.get_param('~map_sizex')+4
		self.col_length = rospy.get_param('~map_sizey')
		self.grid_size = rospy.get_param('~map_resolution')
		self.ttc_th = rospy.get_param('~ttc_th')
		self.view_radar = rospy.get_param('~view_radar')
		self.VEHICLE_AB = rospy.get_param('~vehicle_axis_base')
		self.vehicle_wheel_base = rospy.get_param('~vehicle_wheel_base') / self.grid_size
		self.lateral_clearance = rospy.get_param('~lateral_clearance') / self.grid_size
		self.test_mode = rospy.get_param('~test_mode')
		self.AccResponseTime = rospy.get_param('~AccResponseTime')
		self.BrakeResponseTime = rospy.get_param('~BrakeResponseTime')
		self.StrResponseTime = rospy.get_param('~StrResponseTime')
		self.MAX_LON_ACC = rospy.get_param('~max_lon_acc')
		self.BRAKE_ACC = rospy.get_param('~brake_acc')
		
		self.offset = self.vehicle_wheel_base/2+self.lateral_clearance

		self.started = False
		self.cmd_start = False
		self.my_cmd_start = False
		self.RotationSpd  = 300.0/16.5*np.pi/180.0
		
		self.data = np.zeros((2,2))
		self.flankgrid = np.zeros(4000)
		
		# 列数
		self.col = int(self.col_length/self.grid_size)
		# 行数
		self.row = int(self.row_length/self.grid_size)
		fig = plt.figure(figsize=(7,8))
		ax = plt.axes(xlim=(-5, max(self.col, self.row)+5), ylim=(-50, max(self.col, self.row)+ 5)) 
		
		for i in range(self.col+1):
			plt.plot([i, i], [-40,self.row-40], color='grey')
		for j in range(self.row+1):
			plt.plot([0,self.col], [j-40,j-40], color='grey')
			
		self.line_pre, = ax.plot([],[],'grey', linewidth=3)
		self.traj_dot = ax.scatter([],[],marker='s', linewidth=5, color='green')
		self.dot = ax.scatter([],[],marker='s', linewidth=1, color='r')
		self.flank_dot = ax.scatter([],[],marker='s', linewidth=1, color='r')
		self.path, = ax.plot([],[],'green', linewidth=3)
		self.previewpoint = ax.scatter([],[],marker='s', linewidth=5, color='green')
		if (self.view_radar):
			self.sub = rospy.Subscriber('forward_radar_gridmap', OccupancyGrid, self.callback)
		else:
			self.sub = rospy.Subscriber('gridmap', OccupancyGrid, self.callback)
		
	
		self.cmd_sub = message_filters.Subscriber("loc_planner_cmd", Cmd)
		self.preview_sub = message_filters.Subscriber("preview", Preview)
		ts = message_filters.ApproximateTimeSynchronizer([self.cmd_sub, self.preview_sub], 10, slop=10)
		ts.registerCallback(self.callback_1)
		
		self.path_sub = rospy.Subscriber('path', Path, self.path_callback)
		self.flankgrid_sub = rospy.Subscriber('flank_gridmap', OccupancyGrid, self.flankgrid_callback)

		anim = FuncAnimation(fig, self.timer_callback, interval=20)
		plt.title('show the grid map')
		# plt.axis('off')
		plt.show()


	def flankgrid_callback(self, data):
		self.flankgrid = data.data

	def veh_model(self,v,sita):
		l = (v/3.6 * self.ttc_th) / self.grid_size
		sita = -sita*np.pi / 180.0

		a = [self.col_length*0.5/self.grid_size-self.offset,0]
		b = [l*np.sin(sita)+self.col_length*0.5/self.grid_size-self.offset, l*np.cos(sita)]
		c = [l*np.sin(sita)+self.col_length*0.5/self.grid_size+self.offset, l*np.cos(sita)]
		d = [self.col_length*0.5/self.grid_size+self.offset, 0]

		return [a[0],b[0],c[0],d[0]], [a[1],b[1],c[1],d[1]]

	def veh_traj(self, CurSpd, CurStrAngle, DestSpd, DestStrAngle):
		CurSpd = CurSpd / 3.6
		DestSpd = DestSpd / 3.6
		CurStrAngle = CurStrAngle * np.pi / 180
		DestStrAngle = DestStrAngle * np.pi / 180
		traj = []
		t=0; Heading =0;traj_x=0;traj_y=0;CollisionSpd = CurSpd
		while t < self.ttc_th:
			T = self.grid_size *0.95 /(CollisionSpd+ 1e-8)
			if(T>0.1):
				T=0.1
			t +=T
			if (DestSpd > CollisionSpd):
				if (t < self.AccResponseTime):
					CollisionSpd = CurSpd
				else:
					CollisionSpd += T*self.MAX_LON_ACC
			elif(DestSpd < CollisionSpd):
				if (t < self.BrakeResponseTime):
					CollisionSpd = CurSpd
				else:
					CollisionSpd -= T*self.BRAKE_ACC
			if(t < self.StrResponseTime):
				CollisionAngle= CurStrAngle
			elif(DestStrAngle > CollisionAngle):
				CollisionAngle += T*self.RotationSpd
			elif(DestStrAngle < CollisionAngle):
				CollisionAngle -= T*self.RotationSpd
			CollisionSpd = max(0., CollisionSpd)
			CO =self.VEHICLE_AB/ np.tan(CollisionAngle)
			YawRate= CollisionSpd/CO
			Heading +=YawRate*T
			cos_val= np.cos(Heading)
			sin_val = np.sin(Heading)
		
			traj_x += CollisionSpd *T*cos_val
			traj_y += CollisionSpd *T*sin_val
			traj.append([self.col_length*0.5/self.grid_size-traj_y/self.grid_size-self.offset, traj_x/self.grid_size])
			traj.append([self.col_length*0.5/self.grid_size-traj_y/self.grid_size+self.offset, traj_x/self.grid_size])
			# print(t, traj_x, T, CollisionSpd, DestSpd)
		for i in range(int(2*self.offset)):
			traj.append([self.col_length*0.5/self.grid_size-traj_y/self.grid_size-self.offset+i, traj_x/self.grid_size])
		return np.array(traj)


	def path_callback(self, message):
		path_plot_x = []
		path_plot_y = []
		if message.header.frame_id != "Invalid frame ID":
			for i in range(len(message.poses)):
				if (message.poses[i].pose.position.x>self.row_length) or (message.poses[i].header.frame_id == "Invalid frame ID"):
					path_plot_x.append(message.poses[i].pose.position.x/self.grid_size)
					path_plot_y.append(-1*message.poses[i].pose.position.y/self.grid_size + self.col*0.5)
					break
				else:
					path_plot_x.append(message.poses[i].pose.position.x/self.grid_size)
					path_plot_y.append(-1*message.poses[i].pose.position.y/self.grid_size + self.col*0.5)
		
		self.path.set_data(path_plot_y, path_plot_x)


	def callback(self,message):
		self.data = message.data
		self.started = True


	def callback_1(self, message_cmd, message_pre):
		self.cmd = message_cmd
		self.pre = message_pre
		self.cmd_start = True

	def timer_callback(self,event):
		update = np.where(self.data)[0]

		update_array = np.zeros((len(update),2))
		for i, j in enumerate(update):
			update_array[i] = [0.5+j%self.col, self.row-40-0.5-np.ceil(j/self.col)]
		
		update_flank = np.where(self.flankgrid)[0]

		update_flank_array = np.zeros((len(update_flank),2))
		for i, j in enumerate(update_flank):
			update_flank_array[i] = [0.5+j%self.col, 0.5-np.ceil(j/self.col)]

		self.dot.set_offsets(update_array)
		self.flank_dot.set_offsets(update_flank_array)
		
		if self.cmd_start:
			if (not self.test_mode):
				if self.cmd.header.frame_id == "Invalid frame ID":
					self.traj_dot.set_offsets([0,0])
				else:
					self.traj_dot.set_offsets(self.veh_traj(self.cmd.GPSSpd, 0.0001, self.cmd.DestSpd, (self.cmd.DestStrAngle/16.5)))

				if self.pre.header.frame_id == "Invalid frame ID":
					self.line_pre.set_data(self.veh_model(0, 0))
				else:
					self.line_pre.set_data(self.veh_model(self.pre.speed, (self.pre.SteerAngle/16.5)))
			else:
				self.line_pre.set_data(self.veh_model(self.pre.speed, (self.pre.SteerAngle/16.5)))
				if self.cmd.header.frame_id != "Invalid frame ID":
					self.traj_dot.set_offsets(self.veh_traj(self.cmd.GPSSpd, 0.0001, self.cmd.DestSpd, (self.cmd.DestStrAngle/16.5)))
			
			self.previewpoint.set_offsets((-1*self.pre.y/self.grid_size + 0.5*self.col, self.pre.x/self.grid_size))
		return self.dot

if __name__ == '__main__':
	rospy.init_node('visual_grid_map')
	grid_map()
	rospy.spin()