from rrt import RRT
from generator import Generator
import matplotlib.pyplot as plt
import matplotlib.patches as patches

if __name__ == "__main__":
	gX = int(input("Enter X coordinate of the goal: "))
	gY = int(input("Enter Y coordinate of the goal: "))

	X = int(input("Enter X coordinate of the initial position: "))
	Y = int(input("Enter Y coordinate of the initial position: "))
	n = int(input("Enter the number of obstacles: "))

	start = [X, Y]
	end = [gX, gY]

	generator = Generator(n)
	obstacleList =  generator.generateRect()

	expandDist = 1.3
	goalSample = 0.2

	rrt = RRT(start, end, obstacleList, expandDist, goalSample)
	xPath, yPath, xPlot, yPlot = rrt.planner()

	fig1 = plt.figure()
	ax1 = fig1.add_subplot(111, aspect='equal')
	for (ox, oy, sx, sy) in obstacleList:
		ax1.add_patch(patches.Rectangle((ox - sx/2.0, oy - sy/2.0), sx, sy, color='black'))

	plt.plot(xPath, yPath)
	# plt.scatter(xPlot, yPlot, marker='^', color='red')
	plt.show()