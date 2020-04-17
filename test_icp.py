import open3d as o3d
import numpy as np
import copy
from sys import argv, exit
from PIL import Image


def draw_registration_result(source, target, transformation):
	source_temp = copy.deepcopy(source)
	target_temp = copy.deepcopy(target)
	source_temp.paint_uniform_color([1, 0.706, 0])
	target_temp.paint_uniform_color([0, 0.651, 0.929])
	source_temp.transform(transformation)
	o3d.visualization.draw_geometries([source_temp, target_temp])


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

			if(Z < 0): continue
			if(np.abs(X) > thresh): continue
			if(np.abs(Y) > thresh): continue

			points.append((X, Y, Z))

			colors.append(rgb.getpixel((u, v)))

	points = np.asarray(points)
	colors = np.asarray(colors)

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(points)
	pcd.colors = o3d.utility.Vector3dVector(colors/255)
	downpcd = pcd.voxel_down_sample(voxel_size=0.01)
	downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

	axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
	o3d.visualization.draw_geometries([downpcd, axis])


	return downpcd


if __name__ == "__main__":
	threshold = 0.02

	# srcFile = argv[1]
	# trgFile = argv[2]

	# source = o3d.io.read_point_cloud(srcFile)
	# target = o3d.io.read_point_cloud(trgFile)
	# trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
	# 						 [-0.139, 0.967, -0.215, 0.7],
	# 						 [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
	# print("Initial alignment")
	# draw_registration_result(source, target, trans_init)

	# print("Applying point-to-point ICP")
	# reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
	# 	o3d.registration.TransformationEstimationPointToPoint())
	# draw_registration_result(source, target, reg_p2p.transformation)

	# print("Applying point-to-plane ICP")
	# reg_p2l = o3d.registration.registration_icp(source, target, threshold, trans_init,
	# 	o3d.registration.TransformationEstimationPointToPlane())
	# draw_registration_result(source, target, reg_p2l.transformation)

	focalLength = 617.19
	centerX = 314.647
	centerY = 246.577
	scalingFactor = 1000.0

	srcR = argv[1]
	srcD = argv[2]
	trgR = argv[3]
	trgD = argv[4]

	srcCld = getPointCloud(srcR, srcD)
	trgCld = getPointCloud(trgR, trgD)

	trans_init = np.identity(4)
	draw_registration_result(srcCld, trgCld, trans_init)

	print("Applying point-to-point ICP")
	reg_p2p = o3d.registration.registration_icp(srcCld, trgCld, threshold, trans_init,
		o3d.registration.TransformationEstimationPointToPoint())
	print(reg_p2p)
	draw_registration_result(srcCld, trgCld, reg_p2p.transformation)

	print("Applying point-to-plane ICP")
	reg_p2l = o3d.registration.registration_icp(srcCld, trgCld, threshold, trans_init,
		o3d.registration.TransformationEstimationPointToPlane())
	print(reg_p2l)
	draw_registration_result(srcCld, trgCld, reg_p2l.transformation)