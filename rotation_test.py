import numpy as np
import math
from squaternion import quat2euler, Quaternion
import pyquaternion


if __name__ == '__main__':
	thetaY = math.radians(30)
	Ry = np.array([[math.cos(thetaY), 0, math.sin(thetaY)], [0, 1, 0], [-math.sin(thetaY), 0, math.cos(thetaY)]])
	quat = pyquaternion.Quaternion(matrix = Ry)
	print(quat.elements)

	quat2 = Quaternion(quat.elements[0], quat.elements[1], quat.elements[2], quat.elements[3])
	print(quat2euler(*quat2, degrees=True))