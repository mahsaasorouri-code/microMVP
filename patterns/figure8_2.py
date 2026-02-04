'''
Figure 8 pattern
Follows the function r = cos^2(theta)
'''

import math

def GetPath(num, bound):
	figure8Path = []

	center = ((bound.l + bound.r) / 2, (bound.u + bound.d) / 2)
	radius = 1.5 * bound.height / 2

	theta = math.pi / 2
	numStep = 1000
	step = 2 * math.pi / numStep
	for i in range(numStep):
		thetaFlip = theta if (theta > 3 * math.pi / 2 or theta < math.pi / 2) else 2 * math.pi - theta
		r = radius * math.cos(thetaFlip)**2
		p = (r * math.cos(thetaFlip) + center[0], r * math.sin(thetaFlip) + center[1])
		figure8Path += [p]
		theta = (theta + step) % (2 * math.pi)
	arcLengthPath = arcLengthSplit(figure8Path, 100)
	paths = makePattern(arcLengthPath, num if num % 2 else num + 1)
	print(paths, len(paths))
	print("this is what is returned")
	return paths

# Parametrizes a path by arc length (makes all of the points equidistant)
def arcLengthSplit(path, numSteps):
	distances = [0]
	for p, q in zip(path[:-1], path[1:]):
		distances += [((p[0] - q[0])**2 + (p[1] - q[1])**2)**.5]
	length = sum(distances)
	stepLength = length / numSteps
	newPath = [path[0]]
	nextDist = stepLength
	d = 0
	for i in range(len(distances)):
		d += distances[i]
		if d > nextDist:
			newPath += [path[i]]
			nextDist += stepLength
	return newPath

# given a path, makes n copies offset from each other so the robots can make a pattern
def makePattern(path, numCars):
	carIndex = int(len(path) // numCars)
	paths = []
	for i in range(numCars):
		index = i * carIndex
		paths += [path[index:] + path[:index]]
	return paths
