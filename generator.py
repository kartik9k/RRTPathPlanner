import random
import math

class Generator():
	def __init__(self, n):
		self.n = n
		self.bounds = []
		self.obstacles = []

	def generateRect(self):
		maxSize = ((100 * 100) / (self.n))/2
		for i in range(self.n):
			x, y = random.randint(7, 93), random.randint(7, 93)
			sizeX, sizeY = random.randint(2, int(math.sqrt(maxSize))), random.randint(2, int(math.sqrt(maxSize)))
			self.obstacles.append([x, y, sizeX, sizeY])
		# print self.obstacles
		return self.obstacles