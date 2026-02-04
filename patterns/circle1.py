import math

def GetPath(num, bound):
    paths = [list() for x in range(num)]

    radius = bound.height / 2
    center = ((bound.l + bound.r) / 2, (bound.u + bound.d) / 2)
    step1 = 2 * math.pi / num
    step2 = 2 * math.pi / 50
    
    # 唯一必须修改的语法：print 后面加了括号
    print(radius, center, step1, step2)
    
    for j in range(num):
        angle = j * step1
        x = center[0] + math.cos(angle) * radius
        y = center[1] + math.sin(angle) * radius
        paths[j].append((x, y))
        for i in range(150):
            angle += step2
            x = center[0] + math.cos(angle) * radius
            y = center[1] + math.sin(angle) * radius
            paths[j].append((x, y))
    return paths