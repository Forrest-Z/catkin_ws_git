#! /usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
import message_filters
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

from os.path import isfile
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from help_function import *
from veh_msgs.msg import Search, Preview, GPS, Cmd, Sts

class plotspeed:
	def __init__(self):
		self.time = 0
		self.lasttime = 0
		self.lastDestSpd = 0
		self.lastGPSSpd = 0
		self.lastDestStrAngle = 0
		self.lastCurStrAngle = 0
		self.DestSpd = 0 
		self.GPSSpd = 0
		self.DestStrAngle = 0
		self.CurStrAngle = 0
		self.data2csv = []

		fig = plt.figure()
		self.cout = 0
		self.CurStrAngle = 0
		
		self.sub = rospy.Subscriber("loc_planner_cmd", Cmd, self.callback)
		self.sub_ = rospy.Subscriber("vehicle_sts", Sts, self.callbacksts)
		timer = rospy.Timer(rospy.Duration(1.0 / 10.0), self.timer_callback)
		anim = FuncAnimation(fig, self.plotcallback, interval=20)
		# plt.axis('off')
		plt.show()
		# 将数据写入到csv文件
		dataname = ['DestSpd', 'GPSSpd', 'DestStrAnglem', 'CurStrAngle']
		dataframe = pd.DataFrame(columns=dataname,data=self.data2csv)
		dataframe.to_csv('dest2cur.csv',encoding='gbk')
		print("finish writing csv")

	def plotcallback(self, event):
		pass
	def callbacksts(self, data):
		self.CurStrAngle = data.CurStrAngle

	def callback(self, data):
		self.DestSpd = data.DestSpd 
		self.GPSSpd = data.GPSSpd
		self.DestStrAngle = data.DestStrAngle
		# self.CurStrAngle = data.CurStrAngle
	
	def timer_callback(self, event):
		self.data2csv.append([self.DestSpd, self.GPSSpd, self.DestStrAngle, self.CurStrAngle])
		self.time += 0.1

		plt.subplot(121)
		plt.title("speed")
		plt.plot([self.lasttime, self.time], [self.lastDestSpd, self.DestSpd], 'r')
		plt.plot([self.lasttime, self.time], [self.lastGPSSpd, self.GPSSpd], 'g')
		# plt.xlim((0,50))

		plt.subplot(122)
		plt.title("steerangle")
		plt.plot([self.lasttime, self.time], [self.lastDestStrAngle, self.DestStrAngle], 'r')
		plt.plot([self.lasttime, self.time], [self.lastCurStrAngle, self.CurStrAngle], 'g')
		# plt.xlim((0,50))

		self.lasttime += 0.1
		self.lastDestSpd = self.DestSpd
		self.lastGPSSpd = self.GPSSpd
		self.lastDestStrAngle = self.DestStrAngle
		self.lastCurStrAngle = self.CurStrAngle

def plotoncsv():
	csvdata = pd.read_csv('dest2cur.csv').values
	x = 0
	for i in range(csvdata.shape[0]-1):			 
		plt.subplot(121)
		plt.title("speed")
		plt.plot([x, x+0.1], [csvdata[i][1], csvdata[i+1][1]], 'r')
		plt.plot([x, x+0.1], [csvdata[i][2], csvdata[i+1][2]], 'g')

		plt.subplot(122)
		plt.title("steerangle")
		plt.plot([x, x+0.1], [csvdata[i][3], csvdata[i+1][3]], 'r')
		plt.plot([x, x+0.1], [csvdata[i][4], csvdata[i+1][4]], 'g')

		x += 0.1
	plt.show()


if __name__ == '__main__':
	'''
	rospy.init_node("plotspeed")
	plotspeed()
	rospy.spin()
		
	'''
	if isfile('dest2cur.csv'):
		plotoncsv()
	else:
		rospy.init_node("plotspeed")
		plotspeed()
		rospy.spin()
	
	
