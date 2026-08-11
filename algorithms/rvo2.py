"""
Path Planning Using RVO solver.
Falls back to a straight-line connector when the compiled RVOCaller.exe
binary (Windows-only) isn't available, e.g. on Linux/Mac.
"""
import subprocess
from subprocess import STDOUT, PIPE
import os

pointGrid = list()

def _straight_line_paths(locs, goals, n_steps=20):
    paths = [list() for _ in range(len(locs))]
    for i in range(len(locs)):
        x0, y0 = locs[i]
        x1, y1 = goals[i]
        for s in range(1, n_steps + 1):
            t = s / n_steps
            paths[i].append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return paths

def GetPath(locs, goals, wb, bound):
    paths = [list() for x in range(len(locs))]

    file = open("tempData", "w")
    file.write(str(len(locs)) + " ")
    file.write(str(wb * 1.75) + " ")
    for loc in locs:
        file.write(str(loc[0]) + " " + str(loc[1]) + " ")
    for goal in goals:
        file.write(str(goal[0]) + " " + str(goal[1]) + " ")
    file.close()

    try:
        proc = subprocess.Popen(["algorithms/rvobin/RvoCaller.exe", "tempData"], stdin=PIPE, stdout=PIPE, stderr=STDOUT)
        stdout, stderr = proc.communicate()
    except FileNotFoundError:
        return _straight_line_paths(locs, goals)

    data = stdout.split()
    iterator = 0

    while True:
        try:
            for i in range(len(locs)):
                paths[i].append((float(data[iterator]), float(data[iterator + 1])))
                iterator += 2
        except:
            break

    if not any(paths):
        return _straight_line_paths(locs, goals)

    return paths
