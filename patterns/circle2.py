"""
Multiple small circles.
"""

import math

def GetPath(num, bound):
    paths = [list() for x in range(num)]

    radius = bound.height // 4
    row = int(num + 1) // 2  # 这里最关键，因为报错就出在 row 身上
    xStep = (bound.width - 100) // (row + 1)
    yStep = bound.height // 3
    centers = list()
    for i in range(row):
        centers.append((bound.l + 50 + xStep * (i + 1), bound.u + yStep))
    for i in range(num - row):
        centers.append((bound.l + 50 + xStep * (i + 1), bound.u + 2 * yStep))
    step2 = 2 * math.pi / 30
    angle = 0
    for j in range(150):
        for i in range(num):
            x = centers[i][0] + math.cos(angle) * radius
            y = centers[i][1] + math.sin(angle) * radius
            paths[i].append((x, y))
        angle += step2
    return paths