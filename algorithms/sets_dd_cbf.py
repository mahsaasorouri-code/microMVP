import numpy as np

from env_sets_admm_v5 import Robot, Landmark, EnvSETSv5, CircleObstacle, SquareObstacle
from final_sets_dd_cbf_v2 import SETSPlannerFullyDecent

_R_ROBOT_UNITS = 0.15
_MAX_STEPS = 300
_MIN_STEPS = 20

OBSTACLES_PX = [
    {"type": "circle", "cx": 200,  "cy": 150, "r": 30},
    {"type": "circle", "cx": 400,  "cy": 300, "r": 40},
    {"type": "circle", "cx": 800,  "cy": 450, "r": 50},
    {"type": "circle", "cx": 250,  "cy": 550, "r": 35},
    {"type": "circle", "cx": 1100, "cy": 350, "r": 45},
    {"type": "circle", "cx": 650,  "cy": 600, "r": 30},
    {"type": "circle", "cx": 950,  "cy": 150, "r": 35},
    {"type": "square", "cx": 600,  "cy": 200, "size": 80},
    {"type": "square", "cx": 950,  "cy": 550, "size": 60},
    {"type": "square", "cx": 1050, "cy": 200, "size": 70},
    {"type": "square", "cx": 150,  "cy": 400, "size": 60},
    {"type": "square", "cx": 500,  "cy": 500, "size": 50},
    {"type": "square", "cx": 1200, "cy": 500, "size": 65},
]


def GetPath(locs, goals, wb, bound):
    n = len(locs)
    scale = wb / _R_ROBOT_UNITS

    def to_units(p):
        return np.array([(p[0] - bound.l) / scale, (p[1] - bound.u) / scale], dtype=float)

    def to_px(p):
        return (float(p[0] * scale + bound.l), float(p[1] * scale + bound.u))

    robots = [Robot(to_units(locs[i])) for i in range(n)]
    landmarks = [Landmark(to_units(goals[i])) for i in range(n)]
    for i, r in enumerate(robots):
        r.target = i

    obstacles = []
    for obs in OBSTACLES_PX:
        center_units = to_units((obs["cx"], obs["cy"]))
        if obs["type"] == "circle":
            obstacles.append(CircleObstacle(center_units, obs["r"] / scale))
        elif obs["type"] == "square":
            obstacles.append(SquareObstacle(center_units, obs["size"] / scale))

    env = EnvSETSv5(robots, landmarks, obstacles=obstacles)
    planner = SETSPlannerFullyDecent(env, H=5, K=40, T_gossip=5, K_admm=5, n_branches=2)

    paths_units = [[robots[i].pos.copy()] for i in range(n)]
    arrived = [False] * n

    for step in range(_MAX_STEPS):
        actions, _, _ = planner.plan()
        states = [r.pos.copy() for r in env.robots]
        new_states = env.dynamics(states, actions)

        for i, r in enumerate(env.robots):
            agent = planner.agents[i]
            if arrived[i]:
                pos = agent.rest_pos.copy()
            else:
                pos = new_states[i].copy()
                agent.sense_self(pos)
                if agent.arrived:
                    arrived[i] = True
                    pos = agent.rest_pos.copy()
            r.pos = pos
            paths_units[i].append(pos.copy())

        if all(arrived) and step >= _MIN_STEPS:
            break

    return [[to_px(p) for p in path] for path in paths_units]
