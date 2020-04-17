import numpy as np
import cv2
from matplotlib import pyplot as plt
from sys import argv, exit
import os
import re


def getCorr(imgFile1, imgFile2):
	MIN_MATCH_COUNT = 1

	im1 = cv2.imread(imgFile1, 0)
	im2 = cv2.imread(imgFile2, 0)

	pts_floor = np.array([[180, 299], [460, 290], [585, 443], [66, 462]])
	pts_correct = np.array([[0, 0], [399, 0], [399, 399], [0, 399]])
	homographyMat, status = cv2.findHomography(pts_floor, pts_correct)
	img1 = cv2.warpPerspective(im1, homographyMat, (400, 400))
	img2 = cv2.warpPerspective(im2, homographyMat, (400, 400))

	cv2.imshow("Image1", img1)
	cv2.imshow("Image2", img2)
	cv2.waitKey(0)


	startPts = np.array([[0, 0], [400, 0], [400, 400], [0, 400]])
	endPts = np.array([[400, 400], [0, 400], [0, 0], [400, 0]])
	homoFl, status = cv2.findHomography(startPts, endPts)
	imgFl2 = cv2.warpPerspective(img2, homoFl, (400, 400))

	cv2.imshow("Image1", img1)
	cv2.imshow("Image2", imgFl2)
	cv2.waitKey(0)

	img2 = imgFl2

	sift = cv2.xfeatures2d_SURF.create(20)

	kp1, des1 = sift.detectAndCompute(img1, None)
	kp2, des2 = sift.detectAndCompute(img2, None)

	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks = 50)

	flann = cv2.FlannBasedMatcher(index_params, search_params)

	matches = flann.knnMatch(des1,des2,k=2)

	good = []
	for m,n in matches:
		if m.distance < 0.7*n.distance:
			good.append(m)

	print("No. of Correspondences: {}".format(len(good)))

	if len(good) > MIN_MATCH_COUNT:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
		matchesMask = mask.ravel().tolist()
		draw_params = dict(matchColor = (0,255,0),
						   singlePointColor = None,
						   matchesMask = matchesMask,
						   flags = 2)
		img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
		plt.imshow(img3, 'gray')
		plt.show()

	else:
		print("Not enough matches- %d/%d" % (len(good),MIN_MATCH_COUNT))
		
		matchesMask = None
		draw_params = dict(matchColor = (0,255,0),
						   singlePointColor = None,
						   matchesMask = matchesMask,
						   flags = 2)
		img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
		plt.imshow(img3, 'gray')
		plt.show()

		return None, None


	src_pts = src_pts.squeeze(); dst_pts = dst_pts.squeeze()
	ones = np.ones((src_pts.shape[0], 1))

	src_pts = np.hstack((src_pts, ones))
	dst_pts = np.hstack((dst_pts, ones))

	orgSrc = np.dot(np.linalg.inv(homographyMat), src_pts.T)
	orgDst = np.dot(np.dot(np.linalg.inv(homographyMat), np.linalg.inv(homoFl)), dst_pts.T)

	orgSrc = orgSrc/orgSrc[2, :]
	orgDst = orgDst/orgDst[2, :]

	print("Showing image pairs:")

	imRgb1 = cv2.imread(imgFile1)
	imRgb2 = cv2.imread(imgFile2)

	for i in range(orgSrc.shape[1]):
		im1 = cv2.circle(imRgb1, (int(orgSrc[0, i]), int(orgSrc[1, i])), 5, (0, 0, 255), 3)

	for i in range(orgDst.shape[1]):
		im2 = cv2.circle(imRgb2, (int(orgDst[0, i]), int(orgDst[1, i])), 5, (0, 0, 255), 3)

	cv2.imshow("Image1", im1)
	cv2.imshow("Image2", im2)
	cv2.waitKey(0) 
	cv2.destroyAllWindows()

	return orgSrc, orgDst


def natural_sort(l): 
	convert = lambda text: int(text) if text.isdigit() else text.lower() 
	alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
	return sorted(l, key = alphanum_key)


def readLC(filename):
	f = open(filename, 'r')
	A = f.readlines()
	f.close()

	src, trg = [], []

	for i, line in enumerate(A):
		st, end = line.split(' ')

		src.append(int(st))
		trg.append(int(end.rstrip('\n')))

	return src, trg


if __name__ == '__main__':
	rgbDir = argv[1]
	depthDir = argv[2]
	lcFile = argv[3]

	rgbFiles = os.listdir(rgbDir)
	depthFiles = os.listdir(depthDir)

	rgbSort = natural_sort(rgbFiles)
	depthSort = natural_sort(depthFiles)

	src, trg = readLC(lcFile)

	for i in range(len(src)):
		print("\n---\n")
		print("Pairs: ", src[i], trg[i])
		orgSrc, orgDst = getCorr(rgbDir + rgbSort[src[i]], rgbDir + rgbSort[trg[i]])
	