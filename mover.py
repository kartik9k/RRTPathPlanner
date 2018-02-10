from rrt import RRT
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

	obstacleList = [(5, 5, 2, 2), (7, 7, 1, 1)] 

	expandDist = 0.20
	goalSample = 0.7

	rrt = RRT(start, end, obstacleList, expandDist, goalSample)
	xPath, yPath, xPlot, yPlot = rrt.planner()

	fig1 = plt.figure()
	ax1 = fig1.add_subplot(111, aspect='equal')
	ax1.add_patch(patches.Rectangle((4,4),2,2))
	ax1.add_patch(patches.Rectangle((6.5,6.5),1,1))
	plt.plot(xPlot, yPlot)
	plt.plot(xPath, yPath, color='black')
	plt.show()