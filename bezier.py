import matplotlib.pyplot as plt
import numpy as np


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def print():
        print(x)


def bezierCurve(p0, p1, p2, p3, t):
    xlist = []
    ylist = []
    for i in t:
        x = (1 - i ** 3) * p0.x + 3 * i * (1 - i) ** 2 * p1.x \
            + 3 * i ** 2 * (1 - i) * p2.x + i ** 3 * p3.x
        y = (1 - i ** 3) * p0.y + 3 * i * (1 - i) ** 2 * p1.y \
            + 3 * i ** 2 * (1 - i) * p2.y + i ** 3 * p3.y
        xlist.append(x)
        ylist.append(y)

    return xlist, ylist


def main():
    # Initialize control points
    p0 = Point(0, 0)
    p1 = Point(5, 8)
    p2 = Point(15, 5)
    p3 = Point(10, 10)

    # range of t values
    t = np.arange(0, 1.05, 0.05).tolist()

    # call function
    xl, yl = bezierCurve(p0, p1, p2, p3, t)

    # Plot data
    data = np.array([
        [p0.x, p0.y], [p1.x, p1.y],
        [p2.x, p2.y], [p3.x, p3.y]
    ])
    x, y = data.T
    plt.scatter(x, y)
    plt.plot(xl, yl)
    plt.show()


if __name__ == "__main__":
    main()