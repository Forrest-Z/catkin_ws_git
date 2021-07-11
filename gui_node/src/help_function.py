# -*- coding:utf-8 -*-
# /usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from sympy import *
from Queue import Queue, PriorityQueue

def caculate_weight(node_s, node_g):
	# node_s, node_g 代表node_s 指向 node_g
	# node_s 是 list, 包含 id, x, y, z, limit_v
	distance_x = node_s[1] - node_g[1]
	distance_y = node_s[2] - node_g[2]
	distance_z = node_s[3] - node_s[3]
	# 空间距离
	# weight 还应该包括更多参数
	weight = np.linalg.norm((distance_x, distance_y, distance_z))
	return weight

def graph_add_node(graph, node_s, node_g):
	'''
	# node_s, node_g 代表node_s 指向 node_g
	# node_s 是 list, 包含 id, x, y, z, limit_v
	distance_x = node_s[1] - node_g[1]
	distance_y = node_s[2] - node_g[2]
	distance_z = node_s[3] - node_s[3]
	# 空间距离
	# weight 还应该包括更多参数
	weight = np.linalg.norm((distance_x, distance_y, distance_z))
	'''
	weight = caculate_weight(node_s, node_g)
	if node_s[0] not in graph:
		# graph[node_s[0]] = {node_g[0]: [weight, node_s[4]]}
		graph[node_s[0]] = {node_g[0]: weight}
	else:
		# graph[node_s[0]][node_g[0]] = [weight, node_s[4]]
		graph[node_s[0]][node_g[0]] = weight

	if node_g[0] not in graph:
		graph[node_g[0]] = {node_s[0]: weight}
	else:
		graph[node_g[0]][node_s[0]] = weight
		
	return graph

def heuristic(node,a, b):
	(x1, y1) = node[a][1:3]
	(x2, y2) = node[b][1:3]
	return abs(x1 - x2) + abs(y1 - y2)

def neighbors(graph, node):
	return graph[node]

def cost(graph, from_node, to_node):
	return graph[from_node][to_node]

def a_star_search(graph, node, start, goal):
	frontier = PriorityQueue()
	frontier.put(start, 0)
	came_from = {start: None}
	cost_so_far = {start: 0}

	while not frontier.empty():
		current = frontier.get()

		if current == goal:
			break

		for next in neighbors(graph, current):	# next为索引
			# print(next, "#######")
			new_cost = cost_so_far[current] + cost(graph, current, next)
			# print(new_cost)

			# 如果没有经过下个点或者下个点的cost比上次路径的更小
			if next not in cost_so_far or new_cost < cost_so_far[next]: 
				cost_so_far[next] = new_cost
				priority = new_cost + heuristic(node, goal, next)
				frontier.put(next, priority)
				came_from[next] = current

	return came_from, cost_so_far

def linear(n1, n2, n3, alpha_limit):
	n1 = np.array(n1)
	n2 = np.array(n2)
	n3 = np.array(n3)

	v1 = n2 - n1
	v2 = n3 - n2

	# 夹角 弧度
	theta = np.arccos(np.dot(v1, v2) / ((np.linalg.norm(v1)) * np.linalg.norm(v2)))

	# True代表是一条直线
	if theta < alpha_limit:
		return True
	else:
		return False


def caculate_center(p0, p1, p2, p3):
	r_x = Symbol('r_x')
	r_y = Symbol('r_y')
	
	center_1 = solve([(p2[0]-p1[0])*(r_x-(p1[0]+p2[0])/2)+(p2[1]-p1[1])*(r_y-(p1[1]+p2[1])/2), (r_x-p1[0])*(p1[0]-p0[0])+(r_y-p0[1])*(p1[1]-p0[1])], [r_x, r_y])
	center_2 = solve([(p2[0]-p1[0])*(r_x-(p1[0]+p2[0])/2)+(p2[1]-p1[1])*(r_y-(p1[1]+p2[1])/2), (r_x-p2[0])*(p2[0]-p3[0])+(r_y-p2[1])*(p2[1]-p3[1])], [r_x, r_y])
	# print(p0, p1, p2, p3,center_1, center_2)
	r1 = np.square(center_1[r_x] - p1[0]) + np.square(center_1[r_y] - p1[1])
	r2 = np.square(center_2[r_x] - p2[0]) + np.square(center_2[r_y] - p2[1])
	if r1 >= r2:
		return center_1[r_x], center_1[r_y], np.sqrt(round(r1, 3))
	else:
		return center_2[r_x], center_2[r_y], np.sqrt(round(r2, 3))

def caculate_dot(a,b,c):
	a = np.array(a);b = np.array(b);c = np.array(c)
	v1 = b - a; v2 = c - a
	v3 = b - c; v4 = a - c
	result1 = np.dot(v1, v2)
	result2 = np.dot(v3, v4)
	vec1 = np.dot(v1, v2) * v2 / np.square(np.linalg.norm(v2))
	distance = np.linalg.norm(v1 -vec1)
	return result1 * result2, distance

def interface(p1, p2, p3, p4, alpha_limit):
	if linear(p1, p2, p3, alpha_limit):
		print("#"*10,"it is linear","#"*10)
		return [p2, p3]
	else:
		v1 = np.array(p2) - np.array(p1)
		v2 = np.array(p4) - np.array(p3)
		center_x, center_y, r = caculate_center(p1,p2,p3,p4)

		p = []; circle_x, circle_y = [], []
		for i in range(13):
			# 顺时针取点
			x_ = round(center_x - r * np.cos(30*i/180.0 * np.pi), 3)
			y_ = round(center_y + r * np.sin(30*i/180.0 * np.pi), 3)


			circle_x.append(x_)
			circle_y.append(y_)

			fanwei, distance = caculate_dot(p2, [x_, y_], p3)
			if fanwei > 0 and distance <= r:
				p.append([x_, y_])

		# plt.plot(circle_x,circle_y)
		turn = np.cross(v1, v2)
		if turn > 0:
			# turn left
			p.reverse()

		p.insert(0, p2)
		p.append(p3)

		return p
