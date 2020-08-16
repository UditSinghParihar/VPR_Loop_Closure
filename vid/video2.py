from sys import argv, exit
from PIL import Image
import numpy as np
import os
import re
import math
import matplotlib.pyplot as plt
import cv2


def natural_sort(l): 
	convert = lambda text: int(text) if text.isdigit() else text.lower() 
	alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
	return sorted(l, key = alphanum_key)


def readPose(filename):
	f = open(filename, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for i, line in enumerate(A):
		if(i % 1 == 0):
			(x, y, theta) = line.split(' ')
			X.append(float(x))
			Y.append(float(y))
			THETA.append(math.radians(float(theta.rstrip('\n'))))

	return X, Y, THETA


def readLC(filename):
	f = open(filename, 'r')
	A = f.readlines()
	f.close()

	src, trg, trans = [], [], []

	for i, line in enumerate(A):
		if(i%2 == 0):
			st, end = line.split(' ')
			src.append(int(st)); trg.append(int(end.rstrip('\n')))
		else:
			tran = line.split(' ')
			theta = math.radians(float(tran[2].rstrip('\n')))
			trans.append((float(tran[0]), float(tran[1]), theta))

	return src, trg, trans


def drawLC(X, Y, THETA, srcs, trgs, blk=False):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')
	ax.plot(X[-1], Y[-1], 'bD', markersize=14)

	if(len(srcs) > 0):
		for src, trg in zip(srcs, trgs):
			ax.plot([X[src], X[trg]], [Y[src], Y[trg]], 'b--', markersize=10)
			ax.plot([X[src], X[trg]], [Y[src], Y[trg]], 'bo', markersize=5)

	plt.xlim(10, 35)
	plt.ylim(-15, 10)
	plt.show(block=blk)
	plt.pause(0.000001)
	plt.clf()


def writeG2O(X, Y, THETA, src, trg, trans):
	g2o = open('lessNoise.g2o', 'w')

	for i, (x, y, theta) in enumerate(zip(X, Y, THETA)):
		line = "VERTEX_SE2 " + str(i) + " " + str(x) + " " + str(y) + " " + str(theta)
		g2o.write(line)
		g2o.write("\n")	

	# Odometry
	# T1_w : 1 with respect to world
	g2o.write("# Odometry constraints\n")
	info_mat = "500.0 0.0 0.0 500.0 0.0 500.0"
	for i in range(1, len(X)):
		p1 = (X[i-1], Y[i-1], THETA[i-1])
		p2 = (X[i], Y[i], THETA[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		del_x = str(T2_1[0][2])
		del_y = str(T2_1[1][2])
		del_theta = str(math.atan2(T2_1[1, 0], T2_1[0, 0]))

		line = "EDGE_SE2 "+str(i-1)+" "+str(i)+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat+"\n"
		g2o.write(line)

	# LC Constraints
	g2o.write('# Loop Closure constraints\n')
	info_mat = "700.0 0.0 0.0 700.0 0.0 700.0\n"
	for i in range(len(src)):
		del_x, del_y, del_theta = str(trans[i][0]), str(trans[i][1]), str(trans[i][2])
		line = "EDGE_SE2 "+str(src[i])+" "+str(trg[i])+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat
		g2o.write(line)

	g2o.write("FIX 0\n")
	g2o.close()


# def writeG2O(X, Y, THETA, src, trg, trans):
# 	g2o = open('lessNoise.g2o', 'w')

# 	for i, (x, y, theta) in enumerate(zip(X, Y, THETA)):
# 		line = "VERTEX_SE2 " + str(i) + " " + str(x) + " " + str(y) + " " + str(theta)
# 		g2o.write(line)
# 		g2o.write("\n")	

# 	# Odometry
# 	# T1_w : 1 with respect to world
# 	g2o.write("# Odometry constraints\n")
# 	info_mat = "500.0 0.0 0.0 500.0 0.0 500.0"
# 	for i in range(1, len(X)):
# 		p1 = (X[i-1], Y[i-1], THETA[i-1])
# 		p2 = (X[i], Y[i], THETA[i])
# 		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
# 		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
# 		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
# 		del_x = str(T2_1[0][2])
# 		del_y = str(T2_1[1][2])
# 		del_theta = str(math.atan2(T2_1[1, 0], T2_1[0, 0]))

# 		line = "EDGE_SE2 "+str(i-1)+" "+str(i)+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat+"\n"
# 		g2o.write(line)

# 	# LC Constraints
# 	g2o.write('# Loop Closure constraints\n')
# 	info_mat = "700.0 0.0 0.0 700.0 0.0 700.0\n"
# 	for i in range(len(src)):
# 		del_x, del_y, del_theta = str(trans[i][0]), str(trans[i][1]), str(trans[i][2])
# 		line = "EDGE_SE2 "+str(src[i])+" "+str(trg[i])+" "+del_x+" "+del_y+" "+del_theta+" "+info_mat
# 		g2o.write(line)

# 	g2o.write("FIX 0\n")
# 	g2o.close()


def optimize():
	cmd = "g2o -robustKernel Cauchy -robustKernelWidth 1 -o opt.g2o -i 50 lessNoise.g2o > /dev/null 2>&1"
	os.system(cmd)


def readG2o(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for line in A:
		if "VERTEX_SE2" in line:
			(ver, ind, x, y, theta) = line.split(' ')
			X.append(float(x))
			Y.append(float(y))
			THETA.append(float(theta.rstrip('\n')))

	return (X, Y, THETA)


if __name__ == '__main__':
	rgbDir = argv[1]
	rgbFiles = os.listdir(rgbDir)
	rgbSort = natural_sort(rgbFiles)

	X, Y, THETA = readPose(argv[2])
	src, trg, trans = readLC(argv[3])
	drawLC(X, Y, THETA, src, trg, blk=True)

	odom = []
	for i in range(1, len(X)):
		p1 = (X[i-1], Y[i-1], THETA[i-1])
		p2 = (X[i], Y[i], THETA[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_w = np.array([[math.cos(p2[2]), -math.sin(p2[2]), p2[0]], [math.sin(p2[2]), math.cos(p2[2]), p2[1]], [0, 0, 1]])
		T2_1 = np.dot(np.linalg.inv(T1_w), T2_w)
		odom.append(T2_1)

	srcAnim = []; trgAnim = []; transAnim = []
	XAnim = []; YAnim = []; TAnim = [] 
	XAnim.append(X[0]); YAnim.append(Y[0]); TAnim.append(THETA[0])
	
	for i in range(len(odom)):

		rgb = cv2.imread(rgbDir + rgbSort[i+1])
		cv2.imshow("Image", rgb)
		cv2.waitKey(1)

		p1 = (XAnim[i], YAnim[i], TAnim[i])
		T1_w = np.array([[math.cos(p1[2]), -math.sin(p1[2]), p1[0]], [math.sin(p1[2]), math.cos(p1[2]), p1[1]], [0, 0, 1]])
		T2_1 = odom[i]
		T2_w = np.dot(T1_w, T2_1)
		x = T2_w[0][2]
		y = T2_w[1][2]
		theta = math.atan2(T2_w[1, 0], T2_w[0, 0])
		XAnim.append(x); YAnim.append(y); TAnim.append(theta)

		if((i+1) in trg):
			trgAnim.append(i+1)
			indx = trg.index(i+1)
			srcAnim.append(src[indx])
			transAnim.append(trans[indx])

			# writeG2O(XAnim, YAnim, TAnim, srcAnim, trgAnim, transAnim)
			# optimize()
			# (xOpt, yOpt, tOpt) = readG2o("opt.g2o")
			# XAnim = xOpt; YAnim = yOpt; TAnim = tOpt

		drawLC(XAnim, YAnim, TAnim, srcAnim, trgAnim, blk=False)

		if(i == 0):
			cv2.waitKey(0)


	drawLC(XAnim, YAnim, TAnim, srcAnim, trgAnim, blk=True)
	os.system("rm opt.g2o lessNoise.g2o")