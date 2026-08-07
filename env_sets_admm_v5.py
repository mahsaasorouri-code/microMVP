"""
env_sets_admm_v5.py
===================
Clean environment for SETS-ADMM v17.

Key change from v4:
  - env.dynamics() does NOT call avoid() any more.
  - env.linearize_global() has NO C_g drift term (C_g = 0 everywhere).
  - Obstacle avoidance is handled ENTIRELY by the reactive tangent follower
    in the planner (sets_admm_v17_reactive_tangent.py).

Removing the double-counting (avoid() in dynamics AND CBF in planner)
was causing the two mechanisms to fight each other and produce
collisions instead of preventing them.
"""

import numpy as np


def norm(v):
    return float(np.linalg.norm(v))


def normalize(v):
    n = norm(v)
    if n < 1e-8:
        return np.zeros_like(v, dtype=float)
    return np.array(v, dtype=float) / n


# ------------------------------------------------------------------ #
#  World objects
# ------------------------------------------------------------------ #

class Robot:
    def __init__(self, pos):
        self.pos    = np.array(pos, dtype=float)
        self.target = -1


class Landmark:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)


class CircleObstacle:
    def __init__(self, center, radius):
        self.center = np.array(center, dtype=float)
        self.radius = float(radius)

    def dist(self, p):
        return norm(np.array(p, dtype=float) - self.center) - self.radius

    def normal(self, p):
        return normalize(np.array(p, dtype=float) - self.center)


class SquareObstacle:
    def __init__(self, center, size):
        self.center = np.array(center, dtype=float)
        self.size   = float(size)

    def dist(self, p):
        p = np.array(p, dtype=float)
        d = np.abs(p - self.center) - self.size / 2.0
        return max(d[0], d[1])

    def normal(self, p):
        p   = np.array(p, dtype=float)
        d   = p - self.center
        idx = int(np.argmax(np.abs(d)))
        n   = np.zeros(2, dtype=float)
        n[idx] = np.sign(d[idx]) if abs(d[idx]) > 1e-8 else 1.0
        return n


class StarObstacle:
    def __init__(self, center, outer_radius, inner_radius,
                 num_points=5, rotation=0.0):
        self.center       = np.array(center, dtype=float)
        self.outer_radius = float(outer_radius)
        self.inner_radius = float(inner_radius)
        self.num_points   = int(num_points)
        self.rotation     = float(rotation)

    def vertices(self):
        verts = []
        for k in range(self.num_points * 2):
            ang = self.rotation + k * np.pi / self.num_points
            r   = self.outer_radius if (k % 2 == 0) else self.inner_radius
            verts.append([self.center[0] + r * np.cos(ang),
                          self.center[1] + r * np.sin(ang)])
        return np.array(verts, dtype=float)

    def dist(self, p):
        return norm(np.array(p, dtype=float) - self.center) - self.outer_radius

    def normal(self, p):
        return normalize(np.array(p, dtype=float) - self.center)


# ------------------------------------------------------------------ #
#  Environment
# ------------------------------------------------------------------ #

class EnvSETSv5(object):
    """
    Pure dynamics environment — NO obstacle avoidance in dynamics().
    All avoidance is handled externally (reactive tangent follower).
    """

    def __init__(self, robots, landmarks, obstacles):
        self.robots    = robots
        self.landmarks = landmarks
        self.obstacles = obstacles

        self.dt          = 0.20
        self.coupling    = 0.06
        self.radius      = 1.8
        self.goal_tol    = 0.28

        self.a_ii      = 0.2
        self.a_ij_gain = 0.2
        self.b_i       = 1.0
        self.nx        = 2
        self.nu        = 2

    def n_robots(self):
        return len(self.robots)

    def neighbors(self, i, states):
        return [j for j in range(len(states))
                if j != i and norm(states[i] - states[j]) < self.radius]

    def dynamics(self, states, actions):
        """
        Pure dynamics — NO avoid() call.
        x_{t+1} = x_t + dt * (u_i + coupling * sum_j (x_j - x_i) / d_ij)
        The caller is responsible for ensuring actions are safe.
        """
        new_states = []
        for i in range(len(states)):
            xi       = np.array(states[i], dtype=float)
            ui       = np.array(actions[i], dtype=float)
            coupling = np.zeros(2, dtype=float)
            for j in self.neighbors(i, states):
                diff = np.array(states[j], dtype=float) - xi
                d    = norm(diff)
                if d > 1e-8:
                    coupling += diff / d
            xi_next = xi + self.dt * (ui + self.coupling * coupling)
            new_states.append(xi_next)
        return new_states

    def linearize_global(self, states, actions):
        """
        Linearize the PURE dynamics (no drift from avoid()).
        C_g = 0 everywhere: no more double-counting.
        """
        n  = self.n_robots()
        nx = self.nx
        nu = self.nu

        A_g = np.zeros((n * nx, n * nx))
        B_g = np.zeros((n * nx, n * nu))
        C_g = np.zeros(n * nx)          # always zero in v5

        for i in range(n):
            xi = np.array(states[i], dtype=float)
            ri = slice(i * nx, (i + 1) * nx)
            ci = slice(i * nu, (i + 1) * nu)

            A_g[ri, ri] = (1.0 + self.dt * self.a_ii) * np.eye(nx)

            for j in self.neighbors(i, states):
                xj   = np.array(states[j], dtype=float)
                d_ij = norm(xj - xi)
                if d_ij > 1e-8:
                    rj      = slice(j * nx, (j + 1) * nx)
                    a_ij    = self.a_ij_gain / (d_ij + 1e-9)
                    A_g[ri, rj]  = self.dt * a_ij * np.eye(nx)
                    A_g[ri, ri] -= self.dt * a_ij * np.eye(nx)

            B_g[ri, ci] = self.dt * self.b_i * np.eye(nu)
            # C_g[ri] = 0  (intentional — no avoid() drift)

        return A_g, B_g, C_g
