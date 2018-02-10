import math
import random
import copy

class Node():
	def __init__(self, x, y, par = None):
		self.x = x
		self.y = y
		self.parent = par

class RRT():
	def __init__(self, start, goal, obstacleList, expandDist, goalSampleRate):
		self.start = Node(start[0], start[1])
		self.goal = Node(goal[0], goal[1])
		self.obstacleList = obstacleList
		self.expandDist = expandDist
		self.goalSampleRate = goalSampleRate

	def planner(self):
		self.nodeList = [self.start]
		xPlot = []
		yPlot = []
		while True and self.goal not in self.nodeList:
			if random.random() > self.goalSampleRate:
				rnd = [self.goal.x, self.goal.y]
			else:
				rnd = [random.randint(0, 100), random.randint(0, 100)]

			neareastIndex = self.getNearestIndex(rnd)
			nearestNode = self.nodeList[neareastIndex]
			
			theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

			newNode = copy.deepcopy(nearestNode)
			newNode.x += self.expandDist * math.cos(theta)
			newNode.y += self.expandDist * math.sin(theta)
			newNode.parent = neareastIndex

			if self.doesNotCollide(newNode.x, newNode.y):
				leftx = newNode.x - self.goal.x
				lefty = newNode.y - self.goal.y

				xPlot.append(newNode.x)
				yPlot.append(newNode.y)

				if (math.sqrt(leftx * leftx + lefty * lefty) <= self.expandDist):
					self.goal.parent = neareastIndex
					self.nodeList.append(self.goal)
					print("Goal Reached :D")
				
				else:
					self.nodeList.append(newNode)
			
			else:
				continue

		pathX = []
		pathY = []

		lastInd = len(self.nodeList) - 1
		while self.nodeList[lastInd].parent != None:
			node = self.nodeList[lastInd]
			pathX.append(node.x)
			pathY.append(node.y)
			lastInd = node.parent

		pathX.append(self.start.x)
		pathY.append(self.start.y)
		
		return pathX, pathY, xPlot, yPlot

	def getNearestIndex(self, rnd):
		distances = []
		for node in self.nodeList:
			distances.append((node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2)
		return distances.index(min(distances))

	def doesNotCollide(self, x, y):
		for (ox, oy, dx, dy) in self.obstacleList:
			if (abs(x - ox) <= dx/2.0 and abs(y - oy) <= dy/2.0):
				return False
		return True