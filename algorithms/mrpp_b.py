"""
Path Planning Using MRPP solver.
"""
import utils
import math
import sys
import os
import os.path, subprocess
from subprocess import STDOUT,PIPE
from pprint import pprint

def GetDist(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))



def generateGrid(wb, bound):
    # 1. 强制缩小逻辑边界（不占满 680*480）
    # 我们只在中心 500 * 350 的范围内画图
    SAFE_WIDTH = 500 
    SAFE_HEIGHT = 350
    
    # 计算画布居中的偏移量
    OFFSET_X = (680 - SAFE_WIDTH) / 2   # 约 90
    OFFSET_Y = (480 - SAFE_HEIGHT) / 2  # 约 65
    
    # 2. 重新推导步长
    # 水平最长跨度约 8.5 个单位，垂直约 4.33 个单位
    limit_x = SAFE_WIDTH / 8.5
    limit_y = SAFE_HEIGHT / (5.0 * 0.866)
    
    # 选取最安全的步长
    stepSize = min(limit_x, limit_y)
    
    xSize = 6
    ySize = 6
    utils.gridCopy = [] # 清空全局列表
    
    # 3. 计算起始点 (左上角起始点也根据 OFFSET 移动)
    # 起始点 X = 居中偏移 + 你的基础偏移(25)
    start_x = OFFSET_X + 25 + (stepSize / 2)
    start_y = OFFSET_Y + 20
    
    newLine = [(start_x, start_y)]
    
    while True:
        # 维持原有的 1, 2 交替逻辑
        if len(newLine) % 2 != len(utils.gridCopy) % 2:
            stepSpace = 1
        else:
            stepSpace = 2
            
        next_x = newLine[-1][0] + stepSize * stepSpace
        
        # 强制截断保护：不允许超过 OFFSET_X + SAFE_WIDTH
        max_x_boundary = OFFSET_X + SAFE_WIDTH
        if next_x > max_x_boundary:
            next_x = max_x_boundary
            
        newLine.append((next_x, newLine[0][1]))
        
        if len(newLine) > xSize:
            newLine.pop(-1)
            utils.gridCopy.append(newLine)
            
            if len(utils.gridCopy) == ySize:
                break
                
            # 换行交错逻辑
            nextLineMove = -1 if len(utils.gridCopy) % 2 == 1 else 1
            
            # 计算下一行首点
            next_row_x = newLine[0][0] + (nextLineMove * stepSize / 2)
            # 左侧保护：不允许小于 OFFSET_X
            next_row_x = max(next_row_x, OFFSET_X)
            
            next_row_y = newLine[0][1] + (math.sqrt(3) * stepSize / 2)
            
            newLine = [(next_row_x, next_row_y)]

def assignStart(locs):
    startVertex = []
    # Get the start vertices 
    for x, y in locs:
        closestVertex = -1
        shortestDistace = 10000.0
        # Get the closest vertices
        for i in range(len(utils.gridCopy[0])):
            for j in range(len(utils.gridCopy)):
                thisVertexDist = GetDist(x, y, utils.gridCopy[i][j][0], utils.gridCopy[i][j][1])
                if thisVertexDist < shortestDistace:
                    # If it's already occupied by another car, don't use it
                    if i * 6 + j in startVertex:
                        continue
                    closestVertex = i * 6 + j
                    shortestDistace = thisVertexDist
        startVertex.append(closestVertex)
    return startVertex

def execute_java(startVertex):
    cmd = [
        'java',
        '-Djava.library.path=C:\\gurobi1301\\win64\\bin',
        '-cp', '.;C:\\gurobi1301\\win64\\lib\\gurobi.jar',
        'projects.multipath.ILP.Main', '11', str(len(utils.gridCopy[0])), str(len(utils.gridCopy))
    ]
    for v in startVertex:
        cmd.append(str(v))

    print("[mrpp_b] cmd:", cmd)

    proc = subprocess.Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT, text=True)  # ✅ text=True
    stdout, _ = proc.communicate()

    # 把 Java 输出打印出来，方便以后排查
    print("[mrpp_b] java stdout:\n", stdout)

    return stdout


def extractPath(stdout, startVertex):
    lines = stdout.splitlines()
    pointList = []

    for line in lines:
        line = line.strip()
        if not line:
            continue

        # 你原来用 'Agent' 前缀，保持一致
        if not line.startswith("Agent"):
            continue

        pointList.append([])
        # 这里你的解析方式很“脆”（只适配单字符坐标），但先不动结构
        for j in range(len(line)):
            if line[j] == "(":
                # 你原来是 [j+3] [j+1]，保持
                pointList[-1].append(utils.gridCopy[int(line[j + 3])][int(line[j + 1])])

    return pointList


def GetPath(locs, goals, wb, bound):
    # paths = [list() for x in range(num)]
    if len(utils.gridCopy) == 0: 
        generateGrid(wb, bound)
    startVertex = assignStart(locs)
    stdout = execute_java(startVertex)
    paths = extractPath(stdout, startVertex)  
    pprint(paths)
    return paths

