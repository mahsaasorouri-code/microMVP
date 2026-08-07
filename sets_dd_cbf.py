"""
sets_dd_cbf.py
Wraps the SETS-DD-CBF fully-decentralised planner as a GetPath(locs, goals, wb, bound)
batch planner for the microMVP GUI. Runs a full offline rollout of the CBF-safe
multi-robot controller from the current positions to the given goals, in the
planner's own coordinate scale, then rescales the resulting trajectory back into
microMVP's pixel coordinates.

Requires final_sets_dd_cbf_v2.py and env_sets_admm_v5.py to sit next to qt_gui.py
at the repo root.
"""

import numpy as np

from env_sets_admm_v5 import Robot, Landmark, EnvSETSv5
from final_sets_dd_cbf_v2 import SETSPlannerFullyDecent

_R_ROBOT_UNITS = 0.15
_MAX_STEPS = 300
_MIN_STEPS = 20


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

    env = EnvSETSv5(robots, landmarks, obstacles=[])
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
