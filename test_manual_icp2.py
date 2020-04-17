import numpy as np
import copy
import open3d as o3d
from sys import argv
from PIL import Image
import math


def draw_registration_result(source, target, transformation):
	source_temp = copy.deepcopy(source)
	target_temp = copy.deepcopy(target)
	source_temp.paint_uniform_color([1, 0.706, 0])
	target_temp.paint_uniform_color([0, 0.651, 0.929])
	source_temp.transform(transformation)
	axis1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
	axis2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0.6, 0.2, 0])
	o3d.visualization.draw_geometries([source_temp, target_temp, axis1, axis2])


def draw_registration_result2(source, target, transformation):
	source_temp = copy.deepcopy(source)
	target_temp = copy.deepcopy(target)
	source_temp.paint_uniform_color([1, 0.706, 0])
	target_temp.paint_uniform_color([0, 0.651, 0.929])
	source_temp.transform(transformation)
	trgSph.append(source_temp); trgSph.append(target_temp)
	axis1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
	axis2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
	axis2.transform(transformation)
	trgSph.append(axis1); trgSph.append(axis2)
	o3d.visualization.draw_geometries(trgSph)


def getPointCloud(rgbFile, depthFile):
	thresh = 5.6

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
			if (Z > thresh): continue

			X = (u - centerX) * Z / focalLength
			Y = (v - centerY) * Z / focalLength
			
			Xtemp = X; Ytemp = Y; Ztemp = Z
			X = Ztemp; Y = -Xtemp; Z = -Ytemp

			# if(Z < 0): continue
			# if(np.abs(X) > thresh): continue
			# if(np.abs(Y) > thresh): continue

			points.append((X, Y, Z))
			colors.append(rgb.getpixel((u, v)))

	points = np.asarray(points)
	colors = np.asarray(colors)

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(points)
	pcd.colors = o3d.utility.Vector3dVector(colors/255)
	
	downpcd = pcd
	# downpcd = pcd.voxel_down_sample(voxel_size=0.01)
	# downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

	points = np.asarray(downpcd.points)
	ones = np.ones((points.shape[0], 1))
	points = np.hstack((points, ones))
	thetaY = 0.33163
	Ry = np.array([[math.cos(thetaY), 0, math.sin(thetaY)], [0, 1, 0], [-math.sin(thetaY), 0, math.cos(thetaY)]])
	To_c = np.identity(4) 
	To_c[0, 3] = 0.27
	To_c[2, 3] = 0.54
	To_c[0:3, 0:3] = Ry
	points = To_c @ points.T
	downpcd.points = o3d.utility.Vector3dVector((points.T)[:, 0:3])

	axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
	o3d.visualization.draw_geometries([downpcd, axis])

	return downpcd


def getPointCloud2(rgbFile, depthFile, pts):
	thresh = 5.6

	depth = Image.open(depthFile)
	if depth.mode != "I":
		raise Exception("Depth image is not in intensity format")

	rgb = Image.open(rgbFile)

	points = []
	colors = []

	corIdx = [-1]*len(pts)
	corPts = [None]*len(pts)
	ptIdx = 0
	for v in range(depth.size[1]):
		for u in range(depth.size[0]):
			Z = depth.getpixel((u,v)) / scalingFactor
			if Z==0: continue
			if (Z > thresh): continue

			X = (u - centerX) * Z / focalLength
			Y = (v - centerY) * Z / focalLength
			
			Xtemp = X; Ytemp = Y; Ztemp = Z
			X = Ztemp; Y = -Xtemp; Z = -Ytemp

			# if(Z < 0): continue
			
			points.append((X, Y, Z))
			colors.append(rgb.getpixel((u, v)))

			if((u, v) in pts):
				# print("Point found.")
				index = pts.index((u, v))
				corIdx[index] = ptIdx
				corPts[index] = (X, Y, Z)

			ptIdx = ptIdx+1

	points = np.asarray(points)
	colors = np.asarray(colors)

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(points)
	pcd.colors = o3d.utility.Vector3dVector(colors/255)
	
	# downpcd = pcd
	# downpcd = pcd.voxel_down_sample(voxel_size=0.01)
	# downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

	# points = np.asarray(downpcd.points)
	# ones = np.ones((points.shape[0], 1))
	# points = np.hstack((points, ones))
	# thetaY = 0.33163
	# Ry = np.array([[math.cos(thetaY), 0, math.sin(thetaY)], [0, 1, 0], [-math.sin(thetaY), 0, math.cos(thetaY)]])
	# To_c = np.identity(4) 
	# To_c[0, 3] = 0.27
	# To_c[2, 3] = 0.54
	# To_c[0:3, 0:3] = Ry
	# points = To_c @ points.T
	# downpcd.points = o3d.utility.Vector3dVector((points.T)[:, 0:3])

	# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
	# o3d.visualization.draw_geometries([pcd, axis])

	return pcd, corIdx, corPts


def readCorr(file):
	f = open(file, 'r')
	A = f.readlines()
	f.close()

	X = A[0].split(' '); Y = A[1].split(' ') 
	x = [];	y = []

	for i in range(len(X)):
		x.append(int(float(X[i])))		

	for i in range(len(Y)):
		y.append(int(float(Y[i])))		
	
	pts = []
	for i in range(len(x)):
		pts.append((x[i], y[i]))

	return pts


def getSphere(pts):
	sphs = []

	for ele in pts:
		if(ele is not None):
			sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.04)
			sphere.paint_uniform_color([0.9, 0.2, 0])

			trans = np.identity(4)
			trans[0, 3] = ele[0]
			trans[1, 3] = ele[1]
			trans[2, 3] = ele[2]

			sphere.transform(trans)
			sphs.append(sphere)

	return sphs


def get3dCor(src, trg):
	corr = []

	for sId, tId in zip(src, trg): 
		if(sId != -1 and tId != -1):
			corr.append((sId, tId))

	corr = np.asarray(corr)
	
	return corr


if __name__ == "__main__":
	threshold = 0.02

	focalLength = 617.19
	centerX = 314.647
	centerY = 246.577
	scalingFactor = 1000.0

	srcR = argv[1]
	srcD = argv[2]
	trgR = argv[3]
	trgD = argv[4]
	srcPts = argv[5]
	trgPts = argv[6]

	# srcCld = getPointCloud(srcR, srcD)
	# trgCld = getPointCloud(trgR, trgD)

	# delX = 0.6
	# delY = 0.2
	# delTheta = 3.14
	# trans_init = np.identity(4)
	# Rz = np.array([[math.cos(delTheta), -math.sin(delTheta), 0], [math.sin(delTheta), math.cos(delTheta), 0], [0, 0, 1]])
	# trans_init[0, 3] = delX
	# trans_init[1, 3] = delY
	# trans_init[0:3, 0:3] = Rz

	# draw_registration_result(srcCld, trgCld, trans_init)

	srcPts = readCorr(srcPts)
	trgPts = readCorr(trgPts)

	srcCld, srcIdx, srcCor = getPointCloud2(srcR, srcD, srcPts)
	trgCld, trgIdx, trgCor = getPointCloud2(trgR, trgD, trgPts)

	srcSph = getSphere(srcCor)
	trgSph = getSphere(trgCor)	
	# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
	# srcSph.append(srcCld); srcSph.append(axis)
	# trgSph.append(trgCld); trgSph.append(axis)
	# o3d.visualization.draw_geometries(srcSph)
	# o3d.visualization.draw_geometries(trgSph)

	corr = get3dCor(srcIdx, trgIdx)

	p2p = o3d.registration.TransformationEstimationPointToPoint()
	trans_init = p2p.compute_transformation(srcCld, trgCld, o3d.utility.Vector2iVector(corr))
	draw_registration_result2(srcCld, trgCld, trans_init)