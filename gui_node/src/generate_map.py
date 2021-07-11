#! /usr/bin/env python
# -*- coding: utf-8 -*-
##最小二乘法
import numpy as np   ##科学计算库 
import matplotlib.pyplot as plt  ##绘图库
from scipy.optimize import leastsq  ##引入最小二乘法算法
import pandas as pd

L = 100; D =0.5; theta_limit = 5/180.0 * np.pi
min_num = 20
X, Y = [], []
'''
2018年10月14日 15:19:31
'''
##需要拟合的函数func :指定函数的形状
plt.figure(figsize=(6,5))
def func(p,x):
    k,b=p
    return k*x+b

##偏差函数：x,y都是列表:这里的x,y更上面的Xi,Yi中是一一对应的
def error(p,x,y):
    return func(p,x)-y

def ZXEC(a,b):
	Xi = np.array(X[a:b+1])
	Yi = np.array(Y[a:b+1])
	p0 = [Xi[0],Yi[0]]
	Para = leastsq(error,p0,args=(Xi,Yi))
	l_k, l_b = Para[0]
	return l_k, l_b

def out_range(a, b, last_k):
	l_k, l_b = ZXEC(a,b)
	theta = np.arctan(l_k)
	if last_k != None:
		alpha = np.arctan(last_k)
		if (abs(theta-alpha) > theta_limit) and (b-a > min_num):
			print("too sharp turn!")
			return True

	j = a
	while j < b+1:		
		dist = abs((l_k*X[j]+l_b-Y[j])*np.cos(theta))
		j += 1
		if dist > D:
			print(dist, j, a, b-1)
			return True
	return False

def length_line(a,b):
	vec = np.array(raw_data[b] - raw_data[a])

	length = np.linalg.norm(vec)
	return length

def caculate_cross(i,n,current_k, current_b, last_k, last_b):
	if i == 0:
		point_x = -(current_k*(current_b-Y[i]) - X[i]) / (np.square(current_k)+1)
		point_y = current_k * point_x + current_b
		point = [point_x, point_y, fake_z]
	elif i < sample_num-1:
		cross_x = (current_b - last_b) / (last_k - current_k)
		cross_y = current_k * cross_x + current_b
		
		###########################################################
		#######add when test with the gps data in 2018.11.01#######
		###########################################################
		tem_dis = np.square(cross_x-X[n-1]) + np.square(cross_y - Y[n-1])
		if tem_dis > np.square(D):
			cross_x = X[n-1]
			cross_y = Y[n-1]
			
		point = [cross_x, cross_y, fake_z]
	
	return point


def main():
	global raw_data, fake_z, sample_num
	dataframe = pd.read_csv('../map/raw_data.csv', low_memory=False)
	raw_data = dataframe[lambda dataframe: dataframe['field.header.frame_id'] != "Invalid frame ID"][['field.UTM_x', 'field.UTM_y']].values
	sample_num = len(raw_data)
	fake_z = 10
	
	for i in range(1,sample_num):
		X.append(raw_data[i][0])
		Y.append(raw_data[i][1])
		if i % 20 == 0:
			plt.scatter(X, Y, marker='x')

	
	local_map = {}; k = 1
	# print(sample_num, "@@@@@")
	i = 0; way_points = []
	last_k, last_b = None, None
	while i < sample_num-2:
		n = i + 1
		while True:
			n += 1
			if (n > (sample_num-2)) or out_range(i,n,last_k):
				break
			if (length_line(i, n) > L):
				break
		n -= 1
		current_k, current_b = ZXEC(i,n)
		way_point = caculate_cross(i,n,current_k, current_b, last_k, last_b)
		way_points.append(way_point)
		last_k, last_b = current_k, current_b
		i = n

	final_point = [X[sample_num-2], current_k*X[sample_num-2]+current_b, fake_z]
	way_points.append(final_point)

	# 画图命名
	plt_x = []; plt_y = []
	# 将waypoint放进local_map
	for i in range(len(way_points)-1):
		local_map[k] = [way_points[i], way_points[i+1]]
		plt_x.append(way_points[i][0])
		plt_y.append(way_points[i][1])
		k += 1
	plt_x.append(way_points[-1][0])
	plt_y.append(way_points[-1][1])
	plt.plot(plt_x, plt_y, marker= 'o', color='r')

	print(local_map)

	plt.show()

	with open ('local_map.txt', 'w') as f:
		for k, v in local_map.items():
			f.write('lane %i \n' %k)
			f.write('%s %f %f %f %f \n' %((str(k)+'.'+'1'), local_map[k][0][0], local_map[k][0][1], local_map[k][0][2], 30))
			f.write('%s %f %f %f %f \n' %((str(k)+'.'+'2'), local_map[k][1][0], local_map[k][1][1], local_map[k][0][2], 30))

			if k != 1:
				f.write('%s %s %s \n' %('exit', (str(k-1)+'.'+'2'), (str(k)+'.'+'1')))
		f.write('end ')

if __name__ == "__main__":
	main()
