from sys import argv, exit
import matplotlib.pyplot as plt
import math
import numpy as np
# import os
from random import randint


def readKitti(fileName):
	f = open(fileName, 'r')
	A = f.readlines()
	f.close()

	X = []
	Y = []
	THETA = []

	for line in A:
		l = line.split(' ')
		
		x = float(l[3]); y = float(l[7]); theta = math.atan2(float(l[4]), float(l[0]))
		
		X.append(x)
		Y.append(y)
		THETA.append(theta)

	return (X, Y, THETA)


def draw(X, Y):
	plt.plot(X, Y, 'bo')
	plt.show()


def densify(X, Y, THETA):
	nPoses = 398

	XD, YD, THETAD = [], [], []

	for i in range(nPoses):
		XD.append(X[i]); YD.append(Y[i]); THETAD.append(THETA[i])

		XMid = (X[i] + X[i+1])/2; YMid = (Y[i] + Y[i+1])/2; THETAMid = (THETA[i] + THETA[i+1])/2
		XD.append(XMid); YD.append(YMid); THETAD.append(THETAMid)

		# delX, delY, delTheta = (X[i+1] - X[i])/6, (Y[i+1] - Y[i])/6, (THETA[i+1] - THETA[i])/6

		# X1, Y1, THETA1 = X[i] + delX, Y[i] + delY, THETA[i] + delTheta
		# X2, Y2, THETA2 = X[i] + 2*delX, Y[i] + 2*delY, THETA[i] + 2*delTheta
		# X3, Y3, THETA3 = X[i] + 3*delX, Y[i] + 3*delY, THETA[i] + 3*delTheta
		# X4, Y4, THETA4 = X[i] + 4*delX, Y[i] + 4*delY, THETA[i] + 4*delTheta
		# X5, Y5, THETA5 = X[i] + 5*delX, Y[i] + 5*delY, THETA[i] + 5*delTheta

		# XD.append(X1), YD.append(Y1), THETAD.append(THETA1)
		# XD.append(X2), YD.append(Y2), THETAD.append(THETA2)
		# XD.append(X3), YD.append(Y3), THETAD.append(THETA3)
		# XD.append(X4), YD.append(Y4), THETAD.append(THETA4)
		# XD.append(X5), YD.append(Y5), THETAD.append(THETA5)

	for i in range(nPoses, len(X)):
		XD.append(X[i]); YD.append(Y[i]); THETAD.append(THETA[i])
	
	return XD, YD, THETAD	


def sparsify(X, Y, THETA):
	XS, YS, THETAS = X, Y, THETA
	
	for i in range(0, 66):
		indx = randint(0, len(XS)-1)
		del XS[indx]
		del YS[indx]
		del THETAS[indx]

	return XS, YS, THETAS


def convert(X, Y, THETA):
	A = np.zeros((len(X), 12))

	for i in range(len(X)):
		T = np.identity(4)
		T[0, 3] = X[i]
		T[1, 3] = Y[i]
		R = np.array([[math.cos(THETA[i]), -math.sin(THETA[i]), 0], [math.sin(THETA[i]), math.cos(THETA[i]), 0], [0, 0, 1]])
		T[0:3, 0:3] = R
		
		A[i] = T[0:3, :].reshape(1, 12)

	return A


if __name__ == '__main__':
	(X, Y, THETA) = readKitti(argv[1])

	print(len(X), len(Y), len(THETA))
	draw(X, Y)

	# XD, YD, THETAD = densify(X, Y, THETA)
	XD, YD, THETAD = sparsify(X, Y, THETA); 
	print(len(XD), len(YD), len(THETAD))
	# exit(1)
	draw(XD, YD)

	A = convert(XD, YD, THETAD)

	np.savetxt('sparse_rtabmap2.kitti', A, delimiter=' ')