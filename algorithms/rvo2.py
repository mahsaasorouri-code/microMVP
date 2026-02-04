"""
Path Planning Using RVO solver.
"""
import subprocess
from subprocess import STDOUT,PIPE
import os

pointGrid = list()

def GetPath(locs, goals, wb, bound):
    paths = [list() for x in range(len(locs))]

    # Generate the problem
    file = open("tempData", "w")
    file.write(str(len(locs)) + " ")
    file.write(str(wb * 1.75) + " ") 
    for loc in locs:
        file.write(str(loc[0]) + " " + str(loc[1]) + " ")
    for goal in goals:
        file.write(str(goal[0]) + " " + str(goal[1]) + " ")
    file.close()

    # Call RvoCaller.exe
    proc = subprocess.Popen(["algorithms/rvobin/RvoCaller.exe", "tempData"], stdin = PIPE, stdout=PIPE, stderr=STDOUT)
    stdout, stderr = proc.communicate()
    data = stdout.split()
    iterator = 0

    # Extract path
    while True:
        try:
            for i in range(len(locs)):
                paths[i].append((float(data[iterator]), float(data[iterator + 1])))
                iterator += 2
        except:
            break 

    return paths