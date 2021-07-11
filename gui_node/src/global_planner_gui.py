#! /usr/bin/env python
#-*- coding:utf-8 -*-
#use a_star find global optimal path(286行)
import rospy
import message_filters
import numpy as np
import matplotlib.pyplot as plt
import os

from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from help_function import *
from veh_msgs.msg import Search, Preview, GPS

class Global_planner:
	def __init__(self):
		self.min_x = 1e10; self.max_x = 0.
		self.min_y = 1e10; self.max_y = 0.
		self.start_lane = None
		self.final_lane = None
		self.count = 0
		self.re_search_counts = 1
		self.pub_path = Path()
		self.pub_path.header.frame_id = "Invalid frame ID"
		
		self.smooth_alpha_step = rospy.get_param('~smooth_alpha_step') * np.pi / 180.0
		self.heading_error_th = rospy.get_param('~heading_error_th') * np.pi / 180.0
		self.point2line_dist_th = rospy.get_param('~point2line_dist_th')
		self.mouse_pick_tol = rospy.get_param('~mouse_pick_tol')
		
		self.started = False; self.find_path = False; self.feed_path = False
		self.start_matched = False; self.goal_matched = False
		self.mouse_pick_counts = 0; self.mouse_pick_list = []
		self.plot = None
				
		# read the map file 37到103行
		with open ('../catkin_ws/src/gui_node/src/local_map2.txt', 'r') as f:#打开local_map2.txt文件，此文件中将道路分成很多节线段
			r = 0; c = 0; m = 0
			self.lane = {}; connect = {}; self.node={}
			while True:				   #大循环
				line = f.readline()    #读取local_map2.txt中一行

				row = line.split(' ')  	#将每一行信息按空格分开，放入row中
				if row[0] == 'lane':    #如果行开头为：‘lane’，则row[2]为车道属性：AllowSwitchLane
					r += 1
					# row[2]是车道属性
					self.lane[r] = [row[2]] #将每一段道路属性保存到lane[]中
					# self.lane[r] = ["AllowSwitchLane"]
					continue

				if row[0] == 'exit':    #如果行开头为：‘exit’，则row[1]中放的为上一段的结尾点，row[2]放的是这一段的起始点
					c += 1
					connect[c] = [row[1], row[2]] #将每一段与上一段的连接点信息保存到connect[]中
					continue

				if row[0] == 'mexit': #这个行开头，在道路文件中没有
					# 代表手动添加
					# 找到两条lane的四个点
					m += 1; 
					p1 = self.node[row[1]][1:3]
					p2 = self.node[row[2]][1:3]
					p3 = self.node[row[3]][1:3]
					p4 = self.node[row[4]][1:3]
					inter = interface(p1, p2, p3, p4, self.smooth_alpha_step) # 返回p2、若干个插值、p3
					if len(inter) == 2:
						c += 1
						connect[c] = [row[2], row[3]]
						continue
					else:
						for v in range(1, len(inter)-1):
							waypoint_id = 'h' +str(m)+'.'+str(v)
							self.node[waypoint_id] = [waypoint_id, inter[v][0], inter[v][1], self.node[row[2]][3], self.node[row[2]][4]]

							if v > 1:
								r += 1; self.lane[r] = ["NotAllowSwitchLane"]
								self.lane[r].append(self.node['h'+str(m)+'.'+str(v-1)])
								self.lane[r].append(self.node['h'+str(m)+'.'+str(v)])

						r += 1; self.lane[r] = ["NotAllowSwitchLane"]				
						self.lane[r].append(self.node[row[2]])
						self.lane[r].append(self.node['h'+str(m)+'.'+str(1)])

						r += 1; self.lane[r] = ["NotAllowSwitchLane"]				
						self.lane[r].append(self.node['h'+str(m)+'.'+str(len(inter)-2)])
						self.lane[r].append(self.node[row[3]])
						continue

				if row[0] == 'end':      #如何行开头为：‘end’，则说明整个道路文件读取完毕
					rospy.loginfo('finished in reading the local map')
					break

				# print(line)            #如果开头没有以上的各种情形，则说明这一行就是正常的道路点信息，
				waypoint_id = row[0]; 	 #则row[0]为路点id
				x = float(row[1])		 #则row[1]为此路点的x值
				y = float(row[2])		 #则row[2]为此路点的y值
				z = float(row[3])		 #则row[3]为此路点的z值

				self.min_x = min(x, self.min_x); self.max_x = max(x, self.max_x)
				self.min_y = min(y, self.min_y); self.max_y = max(y, self.max_y)
				limit_v = float(row[4])  #则row[4]为此路点的道路限速值
				self.node[waypoint_id] = [waypoint_id, x, y, z, limit_v]#将一个路点的所有信息放入到node[]中
				self.lane[r].append(waypoint_id)  #然后将此路点的id加在lane[r]的后面
		# print(self.lane)
		# creat the lane graph
		# 节点总数
		self.map_graph = {}
		ratio = (self.max_y - self.min_y) / (self.max_x - self.min_x)
		self.fig = plt.figure(figsize=(4, 4*ratio))
		ax = plt.axes(xlim=(self.min_x-100, self.max_x+100), ylim=(self.min_y-100, self.max_y+100))
		node_sum = 0
		for i in range(1, 1+len(self.lane)):
			x , y =[], []
			node_sum += len(self.lane[i])
			# self.map_graph = graph_add_node(self.map_graph, self.lane[i][1], self.lane[i][2])
			self.map_graph = graph_add_node(self.map_graph, self.node[self.lane[i][1]], self.node[self.lane[i][2]])
			x.append(self.node[self.lane[i][1]][1])
			x.append(self.node[self.lane[i][2]][1])
			y.append(self.node[self.lane[i][1]][2])	
			y.append(self.node[self.lane[i][2]][2])	
			if ("AllowSwitchLane" == self.lane[i][0]):
				plt.plot(x, y, color='blue', lw=3, marker='x')
			else:
				plt.plot(x, y, color='grey', lw=3, marker='x')
			plt.text(x[0], y[0],str(i), fontsize=8)
			
		self.vehpos = ax.scatter([],[],linewidth=8,marker="s",color='b')
			
		for i in range(1, 1+len(connect)):
			c_x, c_y = [], []
			node_s = self.node[connect[i][0]]
			node_g = self.node[connect[i][1]]
			c_x.append(node_s[1])
			c_x.append(node_g[1])
			c_y.append(node_s[2])
			c_y.append(node_g[2])	
			self.map_graph = graph_add_node(self.map_graph, node_s, node_g)
		plt.plot(c_x, c_y, color='grey', lw=3, marker='x')	
		del x, y, c_x, c_y
		# get the goal point	
		#while (not self.goal_matched):
		#	[(pos_x, pos_y), (pos_x1, pos_y1)] = plt.ginput(2,None)
		#	self.goal_match(pos_x, pos_y, pos_x1, pos_y1)

		self.pub = rospy.Publisher('GlobalPath', Path, queue_size = 1)#发布的消息具体格式为在371行，在self.timer_callback函数中
		#消息'GlobalPath'包含：poses和header两部分，poses中包含一系列的：header（包含：seq和frame_id）和pose(包含：position.x，y，z);header中包含：frame_id
		self.timer = rospy.Timer(rospy.Duration(1./20.0), self.timer_callback) #调用timer_callback()函数

		re_search_sub = rospy.Subscriber("re_search", Search, self.search_callback)
		imu_sub = message_filters.Subscriber("IMU_data", Imu)
		pos_sub = message_filters.Subscriber("global_position", GPS)
		ts = message_filters.ApproximateTimeSynchronizer([imu_sub, pos_sub], 1, slop=10)
		ts.registerCallback(self.callback)  #收到‘IMU_data’，和‘global_position’消息后，调用回调函数callback（）

		anim = FuncAnimation(self.fig, self.plot_callback, interval=200)
		self.fig.canvas.mpl_connect('button_press_event', self.mouse_pick)
		# plt.axis('off')
		plt.legend()
		plt.show()

	def goal_match(self, pos_x, pos_y, pos_x1, pos_y1):
		self.final_position = [pos_x, pos_y, 0]
		# print(pos_x1,pos_x, pos_y1,pos_y)
		if (pos_x == None or pos_y == None or pos_x1 == None or pos_y1 == None):
			rospy.logwarn("invalid input")
			return False		
		goal_vec = np.array((pos_x1-pos_x, pos_y1-pos_y))
		vec_y = [0, 1]
		goal_alpha = np.arccos(np.dot(vec_y, goal_vec) / (np.linalg.norm(goal_vec) * np.linalg.norm(vec_y)))
		goal_alpha = (np.cross(vec_y, goal_vec)/(np.linalg.norm(goal_vec)*np.linalg.norm(vec_y)*(np.sin(goal_alpha)+1e-10))) * goal_alpha 
		if goal_alpha < 0:
			goal_alpha = goal_alpha + 2*np.pi
		self.final_heading = goal_alpha
		# print(self.final_heading*180/np.pi,"#%"*50)

		for l in self.lane:
			f_distance, f_parallel, f_direction, f_position = self.map_match(self.lane[l], self.final_position, heading=self.final_heading)
			# rospy.loginfo(f_distance, f_parallel, f_direction,l)
			# print(l, f_distance,f_parallel*180/np.pi,(abs(f_parallel) < self.heading_error_th),(f_distance < self.mouse_pick_tol))
			if (abs(f_parallel) < self.heading_error_th) and (f_distance < self.mouse_pick_tol):
				print("get the fianl lane")
				self.mouse_pick_tol = f_distance
				self.final_direction = f_direction
				self.final_lane = self.lane[l]
				self.goal_pos = f_position
			
		if (self.final_lane == None):
			rospy.logwarn("goal_pos can't be matched: \n %s" %(self.final_lane))
			# return
		else:
			rospy.loginfo("goal_pos has matched: \n %s" %(self.final_lane))
			# 加入当前道路属性
			self.goal_pos.append(0)
			self.goal_pos.append(self.final_lane[0])
			self.goal_matched = True
			return True

	def mouse_pick(self, event):
		self.mouse_pick_counts += 1
		self.mouse_pick_list.append([event.xdata, event.ydata])
		if self.mouse_pick_counts % 2 == 0:
			self.mouse_pick_tol = rospy.get_param('~mouse_pick_tol')
			if self.goal_match(self.mouse_pick_list[0][0],self.mouse_pick_list[0][1],self.mouse_pick_list[1][0], self.mouse_pick_list[1][1]):
				self.start_matched = False
				self.find_path = False
				self.feed_path = False
				# 重新初始化
				self.point2line_dist_th = rospy.get_param('~point2line_dist_th')
				self.start_lane = None
				if (not self.start_matched) and (self.plot != None):			
					self.plot.remove()
			self.mouse_pick_list = []

	def search_callback(self, message):
		#  是否完成规划
		if self.feed_path:
			#  重新规划， 将之前的标志位清空
			#  True 则不需要重规划；	False 则重规划
			if message.Re_Search:
				# 重规划请求
				self.re_search_counts += 1
				if self.re_search_counts % 10 == 0:
					self.re_search_counts = 1
					self.start_matched = False
					self.find_path = False
					self.feed_path = False
					# 重新初始化
					self.point2line_dist_th = rospy.get_param('~point2line_dist_th')
					self.start_lane = None
					if not self.start_matched:			
						self.plot.remove()
					


	def callback(self, data_imu, data_pos):
		if (data_imu.header.frame_id != "Invalid frame ID") and (data_pos.header.frame_id != "Invalid frame ID"):
			self.count = 0

		self.vehicle_orientation = [data_imu.orientation.w,data_imu.orientation.x,data_imu.orientation.y,data_imu.orientation.z]
		# q0, q1, q2, q3 = self.vehicle_orientation
		# print((np.arctan2(2*(q0*q3+q1*q2), (1-2*(q2*q2 + q3*q3))))*180/np.pi)
		self.vehicle_position = [data_pos.UTM_x, data_pos.UTM_y, data_pos.UTM_z]
		self.last_data = data_pos
		# self.vehicle_heading =  data_imu.orientation.w
		if (not self.started):
			self.started = True

		if (not self.start_matched):
			for l in self.lane:
				s_distance, s_parallel, s_direction, _ = self.map_match(self.lane[l], self.vehicle_position, self.vehicle_orientation)
				# print((abs(s_parallel) < self.heading_error_th),(s_distance < self.point2line_dist_th), s_parallel, s_distance)
				# print(l, "!!!!", s_distance)
				# if (abs(s_parallel) < self.heading_error_th) and (s_distance < self.point2line_dist_th):
				if (s_distance < self.point2line_dist_th):
					self.point2line_dist_th = s_distance
					self.start_direction = s_direction
					self.start_lane = self.lane[l]	
			if self.start_lane == None:
				rospy.logwarn("start_pos can't be matched: \n %s" %(self.start_lane))
				return
			else:
				rospy.loginfo("start_pos has matched: \n %s %f" %(self.start_lane, self.point2line_dist_th))
				self.vehicle_position.append(self.start_lane[0])
				self.start_matched = True
				# 由于没有倒车故设定下程序
				self.map_graph[self.start_lane[1]][self.start_lane[2]] = float('inf')
				self.map_graph[self.start_lane[2]][self.start_lane[1]] = float('inf')


		if (self.start_matched and self.goal_matched) and (not self.find_path):
			# direction 0 为同向，1为反方向
			if (self.start_direction):
				start_point = self.start_lane[1]
				complete_path_start = self.node[self.start_lane[2]][1:4]
			else:
				start_point = self.start_lane[2]
				complete_path_start = self.node[self.start_lane[1]][1:4]
			
			complete_path_start.append(self.start_lane[0])

			if (self.final_direction):
				goal_point = self.final_lane[2]
			else:
				goal_point = self.final_lane[1]
			f, s = a_star_search(self.map_graph, self.node, start_point, goal_point) #使用A*算法寻找全局最优路径
			lane_weight = caculate_weight(self.node[self.start_lane[1]], self.node[self.start_lane[2]])
			# print(lane_weight)
			self.map_graph[self.start_lane[1]][self.start_lane[2]] = lane_weight
			self.map_graph[self.start_lane[2]][self.start_lane[1]] = lane_weight
			path = [goal_point]
			while True:
				goal_point = f[goal_point]
				if goal_point == None:
					break
				path.append(goal_point)
			path.reverse()
			print(path, "path with waypoin id")
			# rospy.loginfo("path searched by A star \n %s" %(path))
			
			self.complete_path = [complete_path_start]
			
			for i in range(len(path)-1):
				tempath = []
				tempath.append(self.node[path[i]][1])
				tempath.append(self.node[path[i]][2])
				tempath.append(self.node[path[i]][3])
				for j in range(1, len(self.lane)):
					if ((self.lane[j][1:3] == [path[i], path[i+1]]) or (self.lane[j][1:3] == [path[i+1], path[i]])):
						tempath.append(self.lane[j][0])
						break
				if (len(tempath) == 4):
					self.complete_path.append(tempath)

			tempath = []
			tempath.append(self.node[path[-1]][1])
			tempath.append(self.node[path[-1]][2])
			tempath.append(self.node[path[-1]][3])	
			tempath.append(self.goal_pos[-1])		
			self.complete_path.append(tempath)
			self.complete_path.append(self.goal_pos)

			self.p_x, self.p_y,self.p_z, self.p_lane = [], [], [], []
			for i in range(len(self.complete_path)):
				self.p_x.append(self.complete_path[i][0])
				self.p_y.append(self.complete_path[i][1])
				self.p_z.append(self.complete_path[i][2])
				self.p_lane.append(self.complete_path[i][3])
			
			self.find_path = True
			rospy.loginfo("success in finding path")
			# rospy.loginfo("the complete path with coordinate: \n %s" %(self.complete_path))
			self.plot,  = plt.plot(self.p_x,self.p_y,'r-o')
			
			
		if (self.start_matched and self.goal_matched) and (self.find_path) and (not self.feed_path):
			self.pub_path.poses = []	# 初始化全局路点
			for i in range(len(self.p_x)):
				tem = PoseStamped()  # must be under the cycle
				tem.header.seq = i
				tem.header.frame_id = self.p_lane[i]
				tem.pose.position.x = self.p_x[i]
				tem.pose.position.y = self.p_y[i]
				tem.pose.position.z = self.p_z[i]

				self.pub_path.poses.append(tem)  #

			# 完成全局路径消息填充
			self.feed_path = True
		if self.feed_path:
				self.pub_path.header.frame_id = "lidar_point_front"
		# print(self.pub_path.header.frame_id)

	
	def plot_callback(self, event):
		if self.started:
			self.vehpos.set_offsets([self.last_data.UTM_x, self.last_data.UTM_y])
		return self.vehpos
	
	def timer_callback(self, event):
		if self.started:

			if self.count > 4:
				self.pub_path.header.frame_id = "Invalid frame ID"

			self.count += 1		

			self.pub_path.header.stamp = rospy.get_rostime()		

			# self.pub_path.header.stamp.secs = rospy.get_rostime().secs
			# self.pub_path.header.stamp.nsecs = rospy.get_rostime().nsecs				
			self.pub.publish(self.pub_path)

	def vertical_distance(self, road, x, y, z, sita):
		# ignore z
		st = np.array([self.node[road[1]][1], self.node[road[1]][2]])
		fl = np.array([self.node[road[2]][1], self.node[road[2]][2]])

		position = np.array([x, y])

		line = fl - st
		vec = position - st
		vec_f = position - fl
		# vec1 = np.dot(vec, line) * line / np.square(np.linalg.norm(line))

		r = np.dot(vec, line) / np.square(np.linalg.norm(line))
		vec1 = r * line

		VertIntersection = (vec1 + st).tolist()
		
		if r <= 0:
			# vertical = np.linalg.norm(vec)
			vertical = np.square(vec).sum()
		elif r >= 1:
			# vertical = np.linalg.norm(vec_f)
			vertical = np.square(vec_f).sum()		
		else:
			# vertical = np.linalg.norm(vec - vec1)
			vertical = np.square(vec-vec1).sum()		
		# 车辆朝向的单位向量
		vec_car = [-np.sin(sita), np.cos(sita)]
		# print(vec_car)
		alpha = np.arccos(np.dot(vec_car, line) / (np.linalg.norm(line)))
		beta = np.pi - alpha
		parallel = min(alpha, beta)
		# direction 0 为同向，1　为反方向
		direction = [alpha, beta].index(parallel)
		return vertical, parallel, direction, VertIntersection

	def map_match(self, road, vehicle_position, vehicle_orention = None, heading = None):
		x, y, z = vehicle_position

		if heading == None:
			q0, q1, q2, q3 = vehicle_orention
			# 航向 与y轴正向夹角
			heading = np.arctan2(2*(q0*q3+q1*q2), (1-2*(q2*q2 + q3*q3)))
			# print("heading ##!~!",heading*180/np.pi)
			if heading < 0:
				heading += 2.0 * np.pi

		distance, parallel, direction, VertIntersection = self.vertical_distance(road, x,y,z,heading)

		return distance, parallel, direction, VertIntersection					
			

if __name__ == '__main__':
	# print(os.getcwd())
	rospy.init_node("global_planner_gui", anonymous=True)
	try:
		Global_planner()
		rospy.spin()
	except rospy.ROSInterruptException:
		plt.close()
		pass

	 

