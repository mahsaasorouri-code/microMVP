"""
Path Planning Using MRPP solver.
这个暂时不用了，保留作参考。
"""
import utils
import math
import sys
import os
import munkres
import os.path, subprocess
from subprocess import STDOUT,PIPE
from random import shuffle

def GetDist(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def generateGrid(wb, bound):
    newLine = []
    stepSize = (bound.height) / 5.0 * (2 / math.sqrt(3))
    xSize = 6
    ySize = 6
    newLine.append((bound.l + stepSize / 2, bound.u))    
    while True:
        if len(newLine) % 2 != len(utils.gridCopy) % 2:
            stepSpace = 1
        else:
            stepSpace = 2
        newLine.append((newLine[-1][0] + stepSize * stepSpace, newLine[0][1]))
        if len(newLine) > xSize:
            newLine.pop(-1)
            utils.gridCopy.append(newLine)
            nextLine = []
            if len(utils.gridCopy) % 2 == 1:
                nextLineMove = -1
            else:
                nextLineMove = 1
            nextLine.append((newLine[0][0] + nextLineMove * stepSize / 2, newLine[0][1] + math.sqrt(3) * stepSize / 2))
            if len(utils.gridCopy) == ySize:
                break
            newLine = nextLine

def assignStart(locs):
    """
    locs: [(x,y), ...] in world/UI coords
    return: list of vertex ids (row-major), unique
    vertex_id = row * xSize + col
    """
    if not utils.gridCopy:
        raise RuntimeError("gridCopy is empty; call generateGrid() first")

    ySize = len(utils.gridCopy)          # rows
    xSize = len(utils.gridCopy[0])       # cols

    startVertex = []
    for x, y in locs:
        closestVertex = -1
        shortestDist = float("inf")

        for row in range(ySize):
            for col in range(xSize):
                vx, vy = utils.gridCopy[row][col]  # ✅ row, col
                d = GetDist(x, y, vx, vy)
                vid = row * xSize + col            # ✅ row-major

                if d < shortestDist and vid not in startVertex:
                    closestVertex = vid
                    shortestDist = d

        startVertex.append(closestVertex)

    return startVertex


GUROBI_HOME = r"C:\gurobi1301\win64"
GUROBI_BIN  = os.path.join(GUROBI_HOME, "bin")
GUROBI_JAR  = os.path.join(GUROBI_HOME, "lib", "gurobi.jar")

def execute_java(startVertex, goalVertex):
    """Run Path Planning Algorithm, Return stdout(str)."""
    if len(startVertex) != len(goalVertex):
        raise ValueError(f"startVertex({len(startVertex)}) != goalVertex({len(goalVertex)})")

    cp = f".;{GUROBI_JAR}"  # Windows 用 ; 分隔

    cmd = [
        "java",
        f"-Djava.library.path={GUROBI_BIN}",
        "-cp", cp,
        "projects.multipath.ILP.Main",
        "111",
        str(len(utils.gridCopy[0])),  # xSize
        str(len(utils.gridCopy)),     # ySize
    ]

    n = len(startVertex)
    for i in range(n):
        cmd.append(str(startVertex[i]))
        cmd.append(str(goalVertex[i]))

    print("[mrpp] cmd:", cmd)

    proc = subprocess.Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT, text=True)
    out, _ = proc.communicate()

    print("[mrpp] java stdout:\n", out)
    return out



import re

def extractPath(stdout):
    """
    Parse lines like:
    Agent 0: 0:23(5,3) 1:17(5,2) ...
    and convert (x,y) grid coords into utils.gridCopy[y][x] points.
    """
    pointList = []
    for line in stdout.splitlines():
        line = line.strip()
        if not line.startswith("Agent "):
            continue

        # 只抓 (x,y) 这种括号
        xy = re.findall(r"\((\d+),(\d+)\)", line)
        pts = []
        for x_str, y_str in xy:
            x = int(x_str)
            y = int(y_str)
            # 你的 gridCopy 是 gridCopy[row][col] 还是 gridCopy[col][row]，
            # 你之前用的是 gridCopy[int(..y..)][int(..x..)] or 反过来
            # 根据你原 Java 输出 "(5,3)" 通常理解 x=5,y=3
            pts.append(utils.gridCopy[y][x])
        pointList.append(pts)

    return pointList



def linear_scaling(locs):
    centerx = 640
    centery = 360
    scale_factor = 4 / math.sqrt(3)
    m = munkres.Munkres()
    matrix = [[0 for i in range(9)] for j in range(9)]
    sgs = [(centerx, centery), (centerx + 100, centery), (centerx - 100, centery), (centerx, centery + 100), (centerx, centery - 100), (centerx + 100, centery - 100), (centerx + 100, centery + 100), (centerx - 100, centery - 100), (centerx - 100, centery + 100)]
    for i in range(9):
        for j in range(9):
            matrix[i][j] = GetDist(locs[i][0], locs[i][1], sgs[j][0], sgs[j][1])
    indexes = m.compute(matrix)
    paths0 = [list() for i in range(9)]
    for i in range(9):
        paths0[indexes[i][0]].append(sgs[indexes[i][1]])
        paths0[indexes[i][0]].append(((paths0[indexes[i][0]][0][0] - centerx) * scale_factor + centerx, (paths0[indexes[i][0]][0][1] - centery) * scale_factor + centery))
    new_locs = [0 for i in range(9)]
    for i in range(9):
        new_locs[i] = paths0[i][1]
    startVertex = assignStart(new_locs)
    order = [4, 7, 5, 2, 0, 1, 3, 6, 8]
    goalVertex = [sgs[order[i]] for i in range(9)]
    goalVertex = assignStart(goalVertex)
    stdout = execute_java(startVertex, goalVertex)
    print(stdout)
    paths = extractPath(stdout)
    expected = len(locs)
    if len(paths) != expected:
        raise RuntimeError(f"[mrpp] Java returned {len(paths)} paths (expected 9). Java output:\n{stdout}")
    
    maxLen = 0    
    for i in range(9):
        if len(paths[i]) > maxLen:
            maxLen = len(paths[i])
    for i in range(maxLen):
        for j in range(9):
            if len(paths[j]) <= maxLen:
                paths0[j].append(paths[j][i])
            else:
                paths0[j].append(paths0[j][-1])
    for i in range(9):
        paths0[i].append(((sgs[order[i]][0] - centerx) * scale_factor + centerx, (sgs[order[i]][1] - centery) * scale_factor + centery))
        paths0[i].append(sgs[order[i]])
        paths0[i].append((paths0[i][-1][0], paths0[i][-1][1] - 70))
    return paths0

def GetPath(locs, goals, wb, bound):
    # paths = [list() for x in range(num)]
    if len(utils.gridCopy) == 0: 
        generateGrid(wb, bound)
    # startVertex = assignStart(locs)
    # stdout = execute_java(startVertex)
    # paths = extractPath(stdout)    
    return linear_scaling(locs)