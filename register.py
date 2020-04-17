from sys import argv, exit
from PIL import Image
import numpy as np
import open3d as o3d
import os
import re
import math
import matplotlib.pyplot as plt


def getPointCloud(rgbFile, depthFile, T):
	depth = Image.open(depthFile)
	if depth.mode != "I":
		raise Exception("Depth image is not in intensity format")
	
	rgb = Image.open(rgbFile)

	points = []
	colors = []

	for v in range(depth.size[1]):
		for u in range(depth.size[0]):
			Z = depth.getpixel((u,v)) / scalingFactor
			if Z==0: continue
			if (Z > 4.6): continue

			X = (u - centerX) * Z / focalLength
			Y = (v - centerY) * Z / focalLength
			
			Xtemp = X; Ytemp = Y; Ztemp = Z
			X = Ztemp; Y = -Xtemp; Z = -Ytemp

			if(Z < 0): continue
			if(np.abs(X) > 4.6): continue
			if(np.abs(Y) > 4.6): continue

			points.append((X, Y, Z))

			colors.append(rgb.getpixel((u, v)))

	points = np.asarray(points)
	colors = np.asarray(colors)

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(points)
	pcd.colors = o3d.utility.Vector3dVector(colors/255)

	downpcd = pcd.voxel_down_sample(voxel_size=0.1)
	points = np.asarray(downpcd.points)
	
	ones = np.ones((points.shape[0], 1))
	points = np.hstack((points, ones))


	thetaY = 0.33163
	Ry = np.array([[math.cos(thetaY), 0, math.sin(thetaY)], [0, 1, 0], [-math.sin(thetaY), 0, math.cos(thetaY)]])
	To_c = np.identity(4) 
	To_c[0, 3] = 0.27
	To_c[2, 3] = 0.54
	To_c[0:3, 0:3] = Ry

	points = (T @ To_c) @ points.T

	downpcd.points = o3d.utility.Vector3dVector((points.T)[:, 0:3])

	# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
	# o3d.visualization.draw_geometries([downpcd, axis])

	return downpcd


def natural_sort(l): 
	convert = lambda text: int(text) if text.isdigit() else text.lower() 
	alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
	return sorted(l, key = alphanum_key)


def draw(X, Y, THETA):
	ax = plt.subplot(111)
	ax.plot(X, Y, 'ro')
	ax.plot(X, Y, 'k-')

	plt.show()


def drawTheta(X, Y, THETA):
	ax = plt.subplot(111)

	for i in range(len(THETA)):
		x2 = math.cos(THETA[i]) + X[i]
		y2 = math.sin(THETA[i]) + Y[i]
		plt.plot([X[i], x2], [Y[i], y2], 'm->')

	ax.plot(X, Y, 'ro')
	
	plt.show()


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
			THETA.append(math.radians(float(theta.rstrip('\n'))) )

	return X, Y, THETA


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


def convert(X, Y, THETA):
	A = np.zeros((len(X), 16))

	Tw0 = np.identity(4)
	for i in range(len(X)):
		T = np.identity(4)
		T[0, 3] = X[i]
		T[1, 3] = Y[i]
		R = np.array([[math.cos(THETA[i]), -math.sin(THETA[i]), 0], [math.sin(THETA[i]), math.cos(THETA[i]), 0], [0, 0, 1]])
		T[0:3, 0:3] = R
		
		if(i == 0):
			Tw0 = T

		T = np.linalg.inv(Tw0) @ T
		A[i] = T.reshape(1, 16)
		
	return A


if __name__ == '__main__':
	rgbDir = argv[1]
	depthDir = argv[2]
	poseFile = argv[3]

	rgbFiles = os.listdir(rgbDir)
	depthFiles = os.listdir(depthDir)

	rgbSort = natural_sort(rgbFiles)
	depthSort = natural_sort(depthFiles)

	focalLength = 617.19
	centerX = 314.647
	centerY = 246.577
	scalingFactor = 1000.0

	# X, Y, THETA = readPose(poseFile)
	X, Y, THETA = readG2o(poseFile)
	draw(X, Y, THETA)

	A = convert(X, Y, THETA)

	assCloud = []

	for i in range(0, len(rgbSort), 20):
		pcd = getPointCloud(rgbDir+rgbSort[i], depthDir+depthSort[i], A[i].reshape(4, 4))
		assCloud.append(pcd)

		if(i%1000 == 0):
			print("PointCloud number: ", i)
			o3d.visualization.draw_geometries(assCloud)

	print("Final Registered")
	o3d.visualization.draw_geometries(assCloud)
