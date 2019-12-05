import matplotlib.pyplot as plt
import numpy as np
import math
import operator

'''
This is the Direction class used for
providing each waypoint with a unit direction vector
'''
class Direction:
    i = 0
    j = 0

    def __init__(self, theta):
        self.i = math.cos(math.radians(theta))
        self.j = math.sin(math.radians(theta))

'''
This is the Point class which is used to create
a Point object that has a x position, y position, and
a direction vector.
'''
class Point:
    def __init__(self, x, y, d):
        self.x = x
        self.y = y
        self.d = d

    # Getter methods
    def getX():
        print(x)

    def getY():
        print(y)

'''
Function to calculate a list of potential 
test values for p1 and p2.
'''
def controlPoint(p0, p3):
    diffx = abs(p3.x - p0.x)
    diffy = abs(p3.y - p0.y)
    p1List = []
    p2List = []
    if (diffx == 0):
        diffx = diffy
    if (diffy == 0):
        diffy = diffx

    for i in range(1, 1000, 5):
        p1x = p0.x + (diffx / 3.) * (i / 100) * p0.d.i
        p1y = p0.y + (diffy / 3.) * (i / 100) * p0.d.j
        p2x = p3.x - (diffx / 3.) * (i / 100) * p3.d.i
        p2y = p3.y - (diffy / 3.) * (i / 100) * p3.d.j
        p1List.append(Point(p1x, p1y, 0))
        p2List.append(Point(p2x, p2y, 0))
    return p1List, p2List

'''
Function to determine the bezier curve given 
a list of way points. This function optimizes the 
bezier curve between each pair of waypoints by minimizing
the max curvature over a list of test values for 
p1 and p2.
'''
def bezierCurve(wp, t):
    xfinal = []  # list to store all the x values in final bezier curve
    yfinal = []  # list to store all the y values in final bezier curve

    # Running 'for' loop over all the waypoints to find a bezier
    # curve between each pair of points
    for j in range(0, len(wp) - 1):
        p0 = wp[j]  # current position
        p3 = wp[j + 1]  # final position
        p1List, p2List = controlPoint(p0, p3)
        klist = []

        # Find optimal curve between the given pair of waypoints
        for i in range(0, len(p1List), 1):
            p1 = p1List[i]  # list of potential p1 points
            p2 = p2List[i]  # list of potential p2 points
            K_max = 0  # initialize max curvature to 0

            for p in t:
                # Calculate Curve Points from Q(t)
                x = (pow(1 - p, 3) * p0.x) + (3 * p * pow(1 - p, 2) * p1.x) + (3 * pow(p, 2) * (1 - p) * p2.x) + (
                            pow(p, 3) * p3.x)
                y = (pow(1 - p, 3) * p0.y) + (3 * p * pow(1 - p, 2) * p1.y) + (3 * pow(p, 2) * (1 - p) * p2.y) + (
                            pow(p, 3) * p3.y)
                # Calculate points from Q'(t)
                x1d = 3 * pow(1 - p, 2) * (p1.x - p0.x) + 6 * (1 - p) * p * (p2.x - p1.x) + 3 * pow(p, 2) * (
                            p3.x - p2.x)
                y1d = 3 * pow(1 - p, 2) * (p1.y - p0.y) + 6 * (1 - p) * p * (p2.y - p1.y) + 3 * pow(p, 2) * (
                            p3.y - p2.y)
                # Calculate points from Q''(t)
                x2d = 6 * (1 - p) * (p2.x - 2 * p1.x + p0.x) + 6 * p * (p3.x - 2 * p2.x + p1.x)
                y2d = 6 * (1 - p) * (p2.y - 2 * p1.y + p0.y) + 6 * p * (p3.y - 2 * p2.y + p1.y)

                # Calculate Curvature at t
                k = abs((x2d * y1d + x1d * y2d) / pow(math.sqrt(pow(x1d, 2) + pow(y1d, 2)), 3))

                # Check to see if we have found a new max curvature value
                if (k > K_max):
                    K_max = k  # replace if new value is larger than previous

            # list to keep track of all max curvatures for each pair of p1 & p2
            klist.append(K_max)

            # find min over all max curvatures for p1 & p2
        min_index, min_value = min(enumerate(klist), key=operator.itemgetter(1))
        print("{} {}".format("min curvature is ", min_value))
        p1 = p1List[min_index]  # obtain optimal p1 value
        p2 = p2List[min_index]  # obtain optimal p2 value

        print("waypoint_x: ", p0.x)
        print("waypoint_y: ", p0.y)

        for p in t:
            # Calculate Curve Points from Q(t)
            x = (pow(1 - p, 3) * p0.x) + (3 * p * pow(1 - p, 2) * p1.x) + (3 * pow(p, 2) * (1 - p) * p2.x) + (
                        pow(p, 3) * p3.x)
            y = (pow(1 - p, 3) * p0.y) + (3 * p * pow(1 - p, 2) * p1.y) + (3 * pow(p, 2) * (1 - p) * p2.y) + (
                        pow(p, 3) * p3.y)
            # Calculate points from Q'(t)
            x1d = 3 * pow(1 - p, 2) * (p1.x - p0.x) + 6 * (1 - p) * p * (p2.x - p1.x) + 3 * pow(p, 2) * (p3.x - p2.x)
            y1d = 3 * pow(1 - p, 2) * (p1.y - p0.y) + 6 * (1 - p) * p * (p2.y - p1.y) + 3 * pow(p, 2) * (p3.y - p2.y)
            # Calculate points from Q''(t)
            x2d = 6 * (1 - p) * (p2.x - 2 * p1.x + p0.x) + 6 * p * (p3.x - 2 * p2.x + p1.x)
            y2d = 6 * (1 - p) * (p2.y - 2 * p1.y + p0.y) + 6 * p * (p3.y - 2 * p2.y + p1.y)

            # Calculate Curvature at t
            k = abs((x2d * y1d + x1d * y2d) / pow(math.sqrt(pow(x1d, 2) + pow(y1d, 2)), 3))

            # Calculate and print steering angle
            steering_angle = math.degrees(math.atan(6.3 / (1 / k)))
            print("steering angle: ", steering_angle)

            xfinal.append(x)  # append x points to xfinal
            yfinal.append(y)  # append y points to yfinal

    return xfinal, yfinal


def main():
    # Initialize way points
    waypoints = [Point(804, 227, Direction(180)),
                 Point(503, 369, Direction(-135)),
                 Point(613, 404, Direction(0)),
                 Point(613, 137, Direction(-45)),
                 Point(893, 137, Direction(90)),
                 Point(804, 227, Direction(180))]

    # range of t values
    t = np.arange(0, 1.05, 0.05).tolist()

    # call function
    xl, yl = bezierCurve(waypoints, t)

    # Plot data
    data = np.array([])
    wx = []
    wy = []
    for i in range(0, len(waypoints)):
        wx.append(waypoints[i].x)
        wy.append(waypoints[i].y)

    plt.scatter(wx, wy)
    plt.plot(xl, yl)
    plt.show()


if __name__ == "__main__":
    main()