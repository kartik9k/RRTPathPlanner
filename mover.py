from rrt import RRT
from rrtStar import RRTStar
from generator import Generator
import matplotlib.pyplot as plt
import matplotlib.patches as patches

if __name__ == "__main__":
	print "Preferable goal position ([95-100, 95-100])"
	gX = int(input("Enter X coordinate of the goal: "))
	gY = int(input("Enter Y coordinate of the goal: "))

	print "Preferable goal position ([0-5, 0-5])"
	X = int(input("Enter X coordinate of the initial position: "))
	Y = int(input("Enter Y coordinate of the initial position: "))
	n = int(input("Enter the number of obstacles: "))

	print "Enter your choice: "
	print "1) RRT"
	print "2) RRT Star"

	con = int(input())
	start = [X, Y]
	end = [gX, gY]

	generator = Generator(n)
	obstacleList = generator.generateRect()
	expandDist = 0.7
	goalSample = 0.2

	if con == 1:
		rrt = RRT(start, end, obstacleList, expandDist, goalSample)

	if con == 2:
		itera = n * 1000
		rrt = RRTStar(start, end, obstacleList, expandDist, goalSample, itera)

	xPath, yPath, xPlot, yPlot = rrt.planner()

	if xPath != None:
		fig1 = plt.figure()
		ax1 = fig1.add_subplot(111, aspect='equal')
		for (ox, oy, sx, sy) in obstacleList:
			ax1.add_patch(patches.Rectangle((ox - sx/2.0, oy - sy/2.0), sx, sy, color='black'))

		plt.plot(xPath, yPath)
		plt.scatter(xPlot, yPlot, marker='^', color='red')
		plt.show()