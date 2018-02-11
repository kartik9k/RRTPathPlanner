import math
import random
import copy

class Node():
	def __init__(self, x, y, par = None, cost = 0):
		self.x = x
		self.y = y
		self.cost = cost
		self.parent = par

class RRTStar():
	def __init__(self, start, goal, obstacleList, expandDist, goalSampleRate, maxIter = 3000):
		self.start = Node(start[0], start[1])
		self.goal = Node(goal[0], goal[1])
		self.obstacleList = obstacleList
		self.expandDist = expandDist
		self.goalSampleRate = goalSampleRate
		self.maxIter = maxIter

	def planner(self):
		xPlot = []
		yPlot = []
		self.nodeList = [self.start]
		i = 0
		while True and self.goal not in self.nodeList and i <= self.maxIter:
			i += 1
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
				xPlot.append(xNew)
				yPlot.append(yNew)
				
				newNode = Node(xNew, yNew, neareastIndex, nearestNode.cost + self.expandDist)
				nearInd = self.nearIndex(newNode)
				newNode = self.getConnection(newNode, nearInd)
				self.nodeList.append(newNode)
				self.rewire(newNode, nearInd)			
			else:
				continue

		remDist = []
		for node in self.nodeList:
			remDist.append(math.sqrt((node.x - self.goal.x) ** 2 + (node.y - self.goal.y) ** 2))

		possibleGoalConnections = [remDist.index(i) for i in remDist if i <= self.expandDist]
		if (len(possibleGoalConnections) == 0):
			return None, None, None, None

		lastInd = None
		minCost = min([self.nodeList[i].cost for i in possibleGoalConnections])
		for i in possibleGoalConnections:
			if self.nodeList[i].cost == minCost:
				lastInd = i
				break

		pathX = []
		pathY = []

		while self.nodeList[lastInd].parent != None:
			node = self.nodeList[lastInd]
			pathX.append(node.x)
			pathY.append(node.y)
			lastInd = node.parent

		pathX.append(self.start.x)
		pathY.append(self.start.y)

		return pathX, pathY, xPlot, yPlot

	def rewire(self, newNode, nearInd):
		for ind in nearInd:
			curNode = self.nodeList[ind]

			dx = newNode.x - curNode.x
			dy = newNode.y - curNode.y

			dist = math.sqrt(dx ** 2 + dy ** 2)
			curCost = newNode.cost + dist

			if curNode.cost > curCost:
				if self.isConnectionPossible(newNode, dx, dy):
					curNode.parent = len(self.nodeList) - 1
					curNode.cost = curCost


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

		nnode = len(self.nodeList)
		r = 50.0 * math.sqrt((math.log(nnode) / nnode))
		nearInd = []
		for i in range(len(distances)):
			if distances[i] <= r:
				nearInd.append(i)

		return nearInd

	def getConnection(self,newNode, nearIndex):
		if len(nearIndex) == 0:
			return newNode

		distance = []
		for i in nearIndex:
			dx = newNode.x - self.nodeList[i].x
			dy = newNode.y - self.nodeList[i].y
			d = math.sqrt(dx ** 2 + dy ** 2)
			if self.isConnectionPossible(self.nodeList[i], dx, dy):
				distance.append(self.nodeList[i].cost + d)
			else:
				distance.append(10000)

		minCost = min(distance)
		minInd = nearIndex[distance.index(minCost)]

		if (minCost == 10000):
			return newNode

		newNode.cost = minCost
		newNode.parent = minInd
		return newNode

	def isConnectionPossible(self, node, dx, dy):
		d = math.sqrt(dx ** 2 + dy ** 2)
		iterations = int(d/self.expandDist)

		theta = math.atan2(dy, dx)
		xCurr = node.x
		yCurr = node.y
		for i in range(iterations):
			xCurr += self.expandDist * math.cos(theta)
			yCurr += self.expandDist * math.sin(theta)
			if not self.doesNotCollide(xCurr, yCurr):
				return False

		return True