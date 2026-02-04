"""
Path Planning Algorithms Template
Modify this file to implement/call your own algorithm.
Make copies of this file if you want to generate multiple different patterns.
The files will be automatically loaded into the gui.
The paths are represented as lists of points. 
The points are represented as tuples, e.g. (x, y)
Please make sure that there is no collision in your paths if the cars run in the same speed.
Check circle1.py or circle2.py if you have more questions.

Parameters:
locs: a list of point (x, y) denotes the location of each robot.
wb: wheelbase of the car, similar to the radius of collision detection.
bound: class defines the boundary
    bound.u : upper boundary (y_min)
    bound.d : lower boundary (y_max)
    bound.l : left boundary (x_min)
    bound.r : right boundary (x_max)
    bound.width : bound.r - bound.l
    bound.height : bound.d - bound.u
"""

def GetPath(locs, wb, bound):
    paths = [list() for x in range(num)]

    return paths