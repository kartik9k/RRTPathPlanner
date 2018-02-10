import math
import random
import copy

class Node():
	def __init__(self, x, y, par = None, cost = 0):
		self.x = x
		self.y = y
		self.cost = cost
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

			xNew = nearestNode.x + self.expandDist * math.cos(theta)
			yNew = nearestNode.y + self.expandDist * math.sin(theta)

			if self.doesNotCollide(xNew, yNew):
				newNode = Node(xNew, yNew, neareastIndex, nearestNode.cost + 1)
				
				nearInd = self.nearIndex(newNode)
				newNode = self.getConnection(newNode, nearIndex)
				self.nodeList.append(newNode)
				
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

	def nearIndex(self, newNode):
		distances = []
		for node in self.nodeList:
			distances.append(math.sqrt((node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2))

		r = 3.0 * math.sqrt((math.log(len(self.nodeList)))/ len(self.nodeList))
		nearInd = []
		for i in range(len(distances)):
			if distances[i] <= r:
				nearInd.append(i)

		return nearInd

	def getConnection(self,newNode, nearIndex):
		if len(nearIndex) == 0:
			return newNode

		distance = []
		ind = []
		for i in nearIndex:
			dx = newNode.x - self.nodeList[ind].x
			dy = newNode.y - self.nodeList[ind].y
			d = dx ** 2 + dy ** 2
			if self.isConnectionPossible(self.nodeList[i], dx, dy):
				distance.append(self.nodeList[i].cost + d)
				ind.append(nearIndex.index(i))

		if len(distance) == 0:
			return newNode

		minCost = min(distance)
		minInd = ind[distance.index(min(distance))]

		newNode.cost = minCost
		newNode.parent = minInd

		return newNode

	def isConnectionPossible(node, dx, dy):
		d = dx ** 2 + dy ** 2
		iterations = int(d/self.expandDist)

		xCurr = node.x
		yCurr = node.y
		for i in range(iterations):
			xCurr += self.expandDist * math.cos(theta)
			yCurr += self.expandDist * math.sin(theta)
			if not doesNotCollide(xCurr, yCurr):
				return False

		return True