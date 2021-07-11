#! /usr/bin/env python
# -*- coding: utf-8 -*-
##最小二乘法
import numpy as np   ##科学计算库 
import matplotlib.pyplot as plt  ##绘图库
from scipy.optimize import leastsq  ##引入最小二乘法算法
import pandas as pd
from matplotlib.animation import FuncAnimation

def callback(event):
	pass
def main():
	index_list = []
	min_x = 1e10; max_x = 0.
	min_y = 1e10; max_y = 0.
	# read the map file
	with open ('local_map.txt', 'r') as f:
		r = 0; c = 0; m = 0
		lane = {}; connect = {}; node={}
		while True:
			line = f.readline()
			row = line.split(' ')
			if row[0] == 'lane':
				r += 1
				lane[r] = []
				continue

			if row[0] == 'exit':
				continue

			if row[0] == 'mexit':
				continue

			if row[0] == 'end':
				break

			waypoint_id = row[0]; 
			x = float(row[1])
			y = float(row[2])
			z = float(row[3])
			
			min_x = min(x, min_x); max_x = max(x, max_x)
			min_y = min(y, min_y); max_y = max(y, max_y)
			node[waypoint_id] = [waypoint_id, x, y, z]
			lane[r].append(waypoint_id)

	ratio = (max_y - min_y) / (max_x - min_x)
	fig = plt.figure()
	# fig = plt.figure(figsize=(4, 4*ratio))
	# ax = plt.axes(xlim=(min_x-100, max_x+100), ylim=(min_y-100, max_y+100))
	anim = FuncAnimation(fig, callback, interval=200)
	plot_x, plot_y = [], [] 
	print(lane)
	print(node)


	def mouse_pick(event):
		index = 0; distance_ = 10000
		for i in range(1, len(lane)):
			distance = rough_match(lane[i], event.xdata, event.ydata)
			if distance < distance_:
				distance_ = distance
				index = i
		plot_callback(index)
	
	def rough_match(road, x, y):
		st = np.array([node[road[0]][1], node[road[0]][2]])
		fl = np.array([node[road[1]][1], node[road[1]][2]])

		position = np.array([x, y])

		line = fl - st
		vec = position - st
		vec_f = position - fl
		
		r = np.dot(vec, line) / np.square(np.linalg.norm(line))
		vec1 = r * line
		
		if r <= 0:
			vertical = np.square(vec).sum()
		elif r >= 1:
			vertical = np.square(vec_f).sum()		
		else:
			vertical = np.square(vec-vec1).sum()
		return vertical
		
		
	def plot_callback(index):
		print(index)
		if (0<index<len(lane)):
			if index not in index_list:
				index_list.append(index)
			else:
				index_list.remove(index)
		plt.plot([node[lane[index][0]][1], node[lane[index][1]][1]], [node[lane[index][0]][2], node[lane[index][1]][2]], 'r-o')

	for i in range(1,len(lane)):
		plot_x.append(node[lane[i][0]][1])
		plot_x.append(node[lane[i][1]][1])
		plot_y.append(node[lane[i][0]][2])
		plot_y.append(node[lane[i][1]][2])
	plt.plot(plot_x, plot_y, marker='o')
	fig.canvas.mpl_connect('button_press_event', mouse_pick)
	plt.show()
	print("close")
	index_list.sort()
	print(index_list)

	with open ('local_map.txt', 'r') as f:
		lines = []
		newlines = []
		for line in f.readlines():
			lines.append(line)

	with open ('local_map2.txt', 'w') as f:
		for line in lines:
			if ("lane" == line.split(' ')[0]):
				if (int(line.split(' ')[1]) in index_list):
					line = line.replace("\n", "") + "AllowSwitchLane " + "\n"
				else:
					line = line.replace("\n", "") + "NotAllowSwitchLane " + "\n"
			newlines.append(line)
		# print(newlines)
		f.writelines(newlines)
if __name__ == "__main__":
	main()
