"""
final_sets_dd_cbf.py
=====================
Fully-decentralised SETS-DD-CBF planner, REVISED to match the paper's
Neighborhood-Gramian construction exactly (Section 2.2 / Eq. 14-18,
Algorithm 2.2), as worked out numerically in Proof_of_set-2.pdf.

What changed vs. sets_admm_v30_fully_decent.py:
  - W[i] is now assembled as a genuine block matrix over Si = {i} u N(i),
    with each diagonal/off-diagonal entry a real nx x nx matrix block
    (Wii, Wij), NOT a scalar. Self occupies block-row/column 0, matching
    Eq. (17)'s [Wii, Wij1, ...; Wij1^T, Wj1j1, 0; ...] layout.
  - Eigendecomposition now runs on the (m*nx) x (m*nx) block matrix, and
    the ego action representation is the nx-row block of V[i] sitting at
    this agent's own diagonal position (V_i^[i] in the supervisor's
    notation / Proof_of_set-2.pdf Step 6) -- an nx x k matrix, not a
    single scalar per mode.
  - The MCTS candidate set now uses these ego eigenvectors directly as
    +/- unit directions, exactly Eq. (19)'s Ai = {g_hat} u {+v_q, -v_q},
    instead of routing them through the old scalar c_own re-scaling.
See _build_neighborhood_gramian() and _local_eigendecompose() below for
the core fix.
"""

from __future__ import annotations
import numpy as np

try:
    import cvxpy as cp
    _CVXPY = True
except ImportError:
    _CVXPY = False

from env_sets_admm_v5 import (
    normalize, norm,
    CircleObstacle, SquareObstacle, StarObstacle,
    EnvSETSv5,
)


class RectObstacle:
    def __init__(self, center, half_x, half_y):
        self.center = np.array(center, dtype=float)
        self.half_x = float(half_x)
        self.half_y = float(half_y)

    def dist(self, p):
        p = np.array(p, dtype=float)
        dx = abs(p[0] - self.center[0]) - self.half_x
        dy = abs(p[1] - self.center[1]) - self.half_y
        return max(dx, dy)

_STALL_ACTION_EPS = 0.04
_R_ROBOT_ESCAPE   = 0.05
_OUTWARD_STEP     = 0.30
_REPULSE_R_OBS    = 1.35
_REPULSE_R_ROB    = 0.90
_REPULSE_GAIN_OBS = 0.55
_REPULSE_GAIN_ROB = 0.35
_JUMP_PER_STALL   = 0.04
_JUMP_MAX         = 0.85
_NO_PROGRESS_STEP = 0.05


def _probe_circles(obs, rc=0.12):
    circles = []
    if isinstance(obs, CircleObstacle):
        center = np.asarray(obs.center, dtype=float)
        R_obs  = float(obs.radius)
        circles.append((center, R_obs))
        N = max(8, int(np.ceil(2*np.pi*R_obs / (2*rc))))
        for s in range(N):
            theta = 2*np.pi*s/N
            circles.append((center + R_obs*np.array([np.cos(theta), np.sin(theta)]), rc))
    elif isinstance(obs, SquareObstacle):
        half = obs.size/2.0
        cx, cy = float(obs.center[0]), float(obs.center[1])
        circles.append((np.array([cx, cy]), obs.size/2.0))
        corners = [np.array([cx-half, cy-half]), np.array([cx+half, cy-half]),
                   np.array([cx+half, cy+half]), np.array([cx-half, cy+half])]
        for k in range(4):
            a, b = corners[k], corners[(k+1) % 4]
            L = float(np.linalg.norm(b-a))
            N = max(2, int(np.ceil(L/(2*rc))))
            for s in range(N+1):
                circles.append((a+(s/N)*(b-a), rc))
    elif isinstance(obs, RectObstacle):
        cx, cy = float(obs.center[0]), float(obs.center[1])
        hx, hy = float(obs.half_x), float(obs.half_y)
        circles.append((np.array([cx, cy]), float(np.hypot(hx, hy))))
        corners = [
            np.array([cx - hx, cy - hy]),
            np.array([cx + hx, cy - hy]),
            np.array([cx + hx, cy + hy]),
            np.array([cx - hx, cy + hy]),
        ]
        for k in range(4):
            a, b = corners[k], corners[(k + 1) % 4]
            L = float(np.linalg.norm(b - a))
            N = max(2, int(np.ceil(L / (2 * rc))))
            for s in range(N + 1):
                circles.append((a + (s / N) * (b - a), rc))
    elif isinstance(obs, StarObstacle):
        circles.append((np.asarray(obs.center, dtype=float), obs.inner_radius))
        verts = obs.vertices(); nv = len(verts)
        for k in range(nv):
            a = np.asarray(verts[k], dtype=float)
            b = np.asarray(verts[(k+1) % nv], dtype=float)
            L = float(np.linalg.norm(b-a))
            N = max(1, int(np.ceil(L/(2*rc))))
            for s in range(N+1):
                circles.append((a+(s/N)*(b-a), rc))
    else:
        circles.append((np.asarray(obs.center, dtype=float), float(obs.radius)))
    return circles


def _point_in_polygon(p, verts):
    x, y = float(p[0]), float(p[1]); n = len(verts); inside = False; j = n-1
    for i in range(n):
        xi, yi = float(verts[i][0]), float(verts[i][1])
        xj, yj = float(verts[j][0]), float(verts[j][1])
        if ((yi > y) != (yj > y)) and (x < (xj-xi)*(y-yi)/(yj-yi+1e-15)+xi):
            inside = not inside
        j = i
    return inside


def _obs_dist(obs, p):
    p = np.asarray(p, dtype=float)
    if isinstance(obs, StarObstacle):
        verts = obs.vertices(); nv = len(verts); min_d = float('inf')
        for i in range(nv):
            a = np.asarray(verts[i], dtype=float); b = np.asarray(verts[(i+1) % nv], dtype=float)
            ab = b-a; denom = float(np.dot(ab, ab))
            if denom < 1e-15: continue
            t = float(np.clip(np.dot(p-a, ab)/denom, 0.0, 1.0))
            d = float(np.linalg.norm(p-(a+t*ab)))
            if d < min_d: min_d = d
        return -min_d if _point_in_polygon(p, verts) else min_d
    return obs.dist(p)


def _obs_normal(obs, p):
    p = np.asarray(p, dtype=float)
    d = _obs_dist(obs, p)
    eps2 = 1e-4
    gx = (_obs_dist(obs, p + np.array([eps2, 0.0])) - d) / eps2
    gy = (_obs_dist(obs, p + np.array([0.0, eps2])) - d) / eps2
    g  = np.array([gx, gy])
    nrm = float(np.linalg.norm(g))
    return g/nrm if nrm > 1e-8 else np.array([1.0, 0.0])


def _project_onto_obs_cbf(u_nom, p_i, f_i, obstacles,
                           r_robot, rc, d_margin, gamma,
                           r_threat_obs, n_iter=20):
    u = np.array(u_nom, dtype=float)
    R_base = r_robot + d_margin
    for _ in range(n_iter):
        worst_viol = 0.0; worst_A = None; worst_b = None
        for obs in obstacles:
            for (c_ls, r_ls) in _probe_circles(obs, rc=rc):
                c_ls  = np.asarray(c_ls, dtype=float)
                R_ils = float(r_ls) + R_base
                a_im  = p_i - c_ls
                dist  = float(np.linalg.norm(a_im))
                if dist - R_ils > r_threat_obs: continue
                h_ils = dist**2 - R_ils**2
                A_row = 2.0*a_im
                if np.linalg.norm(A_row) < 1e-8:
                    A_row = 2.0*R_ils*np.array([1.0, 0.0])
                drift_obs = float(np.dot(A_row, f_i))
                rhs_val   = -gamma*h_ils - drift_obs
                viol = rhs_val - float(np.dot(A_row, u))
                if viol > worst_viol:
                    worst_viol = viol; worst_A = A_row; worst_b = rhs_val
        if worst_viol <= 1e-6 or worst_A is None: break
        A_norm_sq = float(np.dot(worst_A, worst_A))
        if A_norm_sq > 1e-12:
            u = u + (worst_b - float(np.dot(worst_A, u)))/A_norm_sq * worst_A
    nm = float(np.linalg.norm(u))
    if nm > 1.0: u = u/nm
    return u


def _solve_robot_qp_local(i, p_i, u_nom, p_ref,
                           nbr_positions, nbr_drifts, nbr_ids,
                           f_i, lam_cbf_i, z_nbr_i,
                           obstacles,
                           d_safe, r_robot, d_margin, rc,
                           r_threat, r_threat_obs,
                           gamma, gamma_clf, p_clf, epsilon):
    R_base = r_robot + d_margin

    diff_clf  = p_i - p_ref
    V_i       = float(np.dot(diff_clf, diff_clf))
    clf_Au    = 2.0*diff_clf
    drift_clf = 2.0*float(np.dot(diff_clf, f_i))
    clf_rhs   = -gamma_clf*V_i - drift_clf
    use_clf   = (V_i > 1e-4)

    obs_A = []; obs_rhs = []
    for obs in obstacles:
        for (c_ls, r_ls) in _probe_circles(obs, rc=rc):
            c_ls  = np.asarray(c_ls, dtype=float)
            R_ils = float(r_ls) + R_base
            a_im  = p_i - c_ls
            dist  = float(np.linalg.norm(a_im))
            if dist - R_ils > r_threat_obs: continue
            h_ils = dist**2 - R_ils**2
            A_row = 2.0*a_im
            if np.linalg.norm(A_row) < 1e-8:
                A_row = 2.0*R_ils*np.array([1.0, 0.0])
            drift_obs = float(np.dot(A_row, f_i))
            rhs_val   = -gamma*h_ils - drift_obs
            if h_ils < 0: rhs_val += -2.0*gamma*h_ils
            obs_A.append(A_row); obs_rhs.append(rhs_val)

    active_nbrs = []; nbr_a_ij = []; nbr_rhs_val = []
    for idx, j in enumerate(nbr_ids):
        p_j  = nbr_positions[idx]
        a_ij = p_i - p_j
        dist = float(np.linalg.norm(a_ij)) + 1e-9
        if dist > r_threat: continue
        h_ij = dist**2 - d_safe**2
        D_ij = float(np.clip(2.0*np.dot(a_ij, f_i - nbr_drifts[idx]), -2.0, 2.0))
        z_j  = z_nbr_i.get(j, 0.0)
        active_nbrs.append(j); nbr_a_ij.append(a_ij)
        nbr_rhs_val.append(gamma*h_ij + D_ij + z_j)

    if not _CVXPY:
        u_star = _project_onto_obs_cbf(
            u_nom, p_i, f_i, obstacles, r_robot, rc, d_margin, gamma, r_threat_obs)
        return u_star, {}

    u     = cp.Variable(2)
    delta = cp.Variable(1)
    n_active = len(active_nbrs)
    z_vars   = cp.Variable(n_active) if n_active > 0 else None

    obj = 0.5*cp.sum_squares(u - u_nom) + p_clf*cp.square(delta)
    for k_idx, j in enumerate(active_nbrs):
        a_ij      = nbr_a_ij[k_idx]
        h_ij      = float(np.linalg.norm(a_ij))**2 - d_safe**2
        D_ij      = float(np.clip(2.0*np.dot(a_ij, f_i - nbr_drifts[nbr_ids.index(j)]), -2.0, 2.0))
        z_j_fixed = z_nbr_i.get(j, 0.0)
        lam_i     = lam_cbf_i.get(j, 0.0)
        g_ij = (-2.0*a_ij@u - gamma*h_ij - D_ij + z_vars[k_idx] - z_j_fixed)
        obj += lam_i*g_ij + epsilon*cp.square(z_vars[k_idx])

    cons = [delta >= 0.0]
    if use_clf:
        cons.append(clf_Au@u - delta <= clf_rhs)
    obs_slack = cp.Variable(len(obs_A), nonneg=True) if obs_A else None
    for k in range(len(obs_A)):
        s_k = obs_slack[k] if obs_slack is not None else 0.0
        obj = obj + 1000.0*s_k
        cons.append(obs_A[k]@u + s_k >= obs_rhs[k])
    z_max = 2.0
    if z_vars is not None:
        for k_idx in range(n_active):
            cons.append(-2.0*nbr_a_ij[k_idx]@u + z_vars[k_idx] <= nbr_rhs_val[k_idx])
            cons.append(z_vars[k_idx] >= -z_max)
            cons.append(z_vars[k_idx] <=  z_max)

    prob = cp.Problem(cp.Minimize(obj), cons)
    try:
        prob.solve(solver=cp.OSQP, warm_start=True,
                   eps_abs=1e-5, eps_rel=1e-5, max_iter=20000,
                   polish=True, verbose=False)
    except Exception:
        pass

    if u.value is None or prob.status in ('infeasible', 'unbounded'):
        u_star = _project_onto_obs_cbf(
            u_nom, p_i, f_i, obstacles, r_robot, rc, d_margin, gamma, r_threat_obs)
        if float(np.linalg.norm(u_star)) < 0.05:
            nm_raw = float(np.linalg.norm(u_nom))
            u_star = np.array(u_nom)/nm_raw if nm_raw > 1e-8 else np.array([1., 0.])
        return u_star, {}

    u_star = np.array(u.value, dtype=float)
    z_new  = {}
    if z_vars is not None and z_vars.value is not None:
        for k_idx, j in enumerate(active_nbrs):
            z_new[j] = float(np.clip(z_vars.value[k_idx], -20.0, 20.0))
    return u_star, z_new


class _MCTSNode:
    """One node of a per-agent Monte-Carlo planning tree: a predicted
    (depth, position) reached by a specific sequence of candidate
    directions from this robot's current state. Field names match the
    convergence-proof notation directly -- T is the visit count T(i, l),
    V is the running backed-up value V(i, l) of node i after l search
    iterations."""
    __slots__ = ('pos', 'depth', 'parent', 'action', 'reward',
                 'children', 'untried', 'T', 'W', 'V')

    def __init__(self, pos, depth, parent, action, reward, untried):
        self.pos      = pos
        self.depth    = depth
        self.parent   = parent
        self.action   = action      # index into the shared action set that produced this node
        self.reward   = reward      # one-step reward earned arriving here (0.0 for the root)
        self.children = {}          # action_idx -> _MCTSNode
        self.untried  = list(untried)
        self.T = 0                  # visit count   T(i, l)
        self.W = 0.0                # running sum of backed-up returns
        self.V = 0.0                # running mean  V(i, l) = W / T


class RobotAgentFullyDecent:

    def __init__(self, robot_id, n_robots, goal_pos, obstacles, params):
        self.i         = robot_id
        self.n         = n_robots
        self.goal      = np.asarray(goal_pos, dtype=float)
        self.obstacles = obstacles
        self.p         = params

        self.pos      = np.zeros(2)
        self.arrived  = False
        self.rest_pos = None

        self.c_own   = np.zeros(params['H'] * params['nu'])
        self.eigvals = np.zeros(4)
        # V_i^[i] in the supervisor's notation: this agent's own nx-row
        # block of the local eigenvector matrix V[i] (Proof_of_set-2.pdf
        # Step 6/7). nx rows (its own state dimension), 4 columns (the
        # four retained joint eigenmodes). Genuinely 2-D per mode, not a
        # scalar -- this is what Eq. (19)'s v_q^(i) actually is.
        self.eigvecs_ego = np.zeros((params['nx'], 4))

        # --- block-Gramian state, matching Eq. (14)-(18) / Alg. 2.2 ---
        # own_var: the scalar magnitude of this robot's own diagonal
        #   Gramian block. Because the surrogate dynamics used here are
        #   isotropic (Aii = a_ii*I_nx, Bi = b_i*I_nx), Wii collapses
        #   exactly to own_var * I_nx -- own_var IS Wii's real value,
        #   not an approximation of a matrix by a scalar.
        # _nbr_sens[j]: this robot's own one-sided estimate of Sij, from
        #   which the coupling block Wij = 0.5*(Sij+Sji) * I_nx is built
        #   -- again exact under the isotropic surrogate, per the same
        #   Jacobian entering the drift term.
        # W_local: the genuine (1+|N(i)|)*nx x (1+|N(i)|)*nx block matrix
        #   W[i] built fresh each planning step from this round's
        #   broadcast -- agent i's own block sits at block-row/column 0,
        #   its neighbours' diagonal blocks follow, off-diagonal blocks
        #   are zero for any pair that is not a direct edge (including
        #   neighbour-to-neighbour pairs this agent cannot see).
        self.own_var    = 0.0
        self._nbr_sens: dict[int, float] = {}
        nx0 = params['nx']
        self.W_local       = np.zeros((nx0, nx0))
        self._gramian_ids  = [robot_id]

        self.lam_cbf: dict[int, float] = {}
        self.z_out:   dict[int, float] = {}
        self.z_in:    dict[int, float] = {}

        self._prev_pos      = None
        self._stall_count   = 0
        self._prev_goal_dist = None
        self._no_progress   = 0
        self._prev_escape   = None
        self._STALL_THRESH  = 0.03
        self._STALL_STEPS   = 3

        self.safe_goal = self._compute_safe_goal(goal_pos, obstacles, params)

        self._inbox: dict = {}
        self.action = np.zeros(params['nu'])

    def _compute_safe_goal(self, goal_pos, obstacles, params):
        goal = np.asarray(goal_pos, dtype=float)
        R_base = params['r_robot'] + params['d_margin']
        for _ in range(40):
            worst_penetration = 0.0
            worst_dir = None
            for obs in obstacles:
                for (c_ls, r_ls) in _probe_circles(obs, rc=params['rc']):
                    c_ls  = np.asarray(c_ls, dtype=float)
                    R_eff = float(r_ls) + R_base
                    diff  = goal - c_ls
                    dist  = float(np.linalg.norm(diff)) + 1e-9
                    penetration = R_eff - dist
                    if penetration > worst_penetration:
                        worst_penetration = penetration
                        worst_dir = diff / dist
            if worst_penetration <= 0.01 or worst_dir is None:
                break
            goal = goal + (worst_penetration + 0.05) * worst_dir
        return goal

    def get_outbox(self) -> dict:
        """What this robot transmits. No agent ever reads another agent's
        attributes directly -- everything crosses the wire through here."""
        return {
            'pos'     : self.pos.copy(),
            'arrived' : self.arrived,
            'own_var' : float(self.own_var),
            'nbr_sens': dict(self._nbr_sens),
            'z_out'   : dict(self.z_out),
            'drift'   : self._local_drift(),
            'degree'  : len(self._inbox.get('nbr_states', {})),
        }

    def receive(self, all_outboxes: list[dict], sensing_radius: float):
        """Every transmitted packet reaches every robot (free-space
        broadcast); each robot decides for itself, using only its own
        position and the sender's published position, which packets are
        within range. Neighbour detection is therefore local, not a
        service the router performs on the robot's behalf."""
        nbr_states={}; nbr_arrived={}; nbr_own_var={}; nbr_sens_map={}; nbr_z={}
        nbr_drifts={}; nbr_degree={}
        for j, pkt in enumerate(all_outboxes):
            if j == self.i:
                continue
            if float(np.linalg.norm(self.pos - pkt['pos'])) < sensing_radius:
                nbr_states[j]   = pkt['pos']
                nbr_arrived[j]  = pkt['arrived']
                nbr_own_var[j]  = pkt['own_var']
                nbr_sens_map[j] = pkt['nbr_sens']
                nbr_z[j]        = pkt['z_out']
                nbr_drifts[j]   = pkt['drift']
                nbr_degree[j]   = pkt['degree']
        self._inbox = {
            'nbr_states'  : nbr_states,
            'nbr_arrived' : nbr_arrived,
            'nbr_own_var' : nbr_own_var,   # neighbour j's own W_jj (real, not consensus)
            'nbr_sens_map': nbr_sens_map,  # neighbour j's own coupling row (for symmetrising W_ij)
            'nbr_z'       : nbr_z,
            'nbr_drifts'  : nbr_drifts,
            'nbr_degree'  : nbr_degree,
        }

    def sense_self(self, measured_pos: np.ndarray):
        """The one piece of privileged information every robot is entitled
        to without communicating: its own measured position (GPS/odometry),
        and its own arrival status against its own a-priori-known goal.
        rest_pos records exactly which point triggered arrival -- the
        literal landmark when it's reachable, the CBF-cleared safe_goal
        substitute when the literal landmark sits inside an obstacle's
        margin -- so nothing downstream has to re-derive or guess it."""
        self.pos = np.asarray(measured_pos, dtype=float).copy()
        at_goal      = float(np.linalg.norm(self.pos - self.goal))      < self.p['goal_tol']
        at_safe_goal = float(np.linalg.norm(self.pos - self.safe_goal)) < self.p['goal_tol']
        self.arrived = at_goal or at_safe_goal
        if at_goal:
            self.rest_pos = self.goal.copy()
        elif at_safe_goal:
            self.rest_pos = self.safe_goal.copy()

    def _local_linearise(self):
        """Local replacement for env.linearize_global(). Robot i rolls its
        own predicted position forward H steps under a fixed nominal action,
        using the same surrogate dynamics constants (a_ii, a_ij_gain, b_i)
        the global linearizer used -- but evaluated from self.pos and the
        neighbour positions already sitting in the inbox, nothing else.
        Neighbours are held frozen for the horizon (their own action was
        already assumed to be zero in the original global rollout; holding
        their position fixed too is the natural decentralised reading of
        that same assumption). Under that reading, neighbour coupling acts
        as a known additive disturbance on robot i's own trajectory rather
        than a state that co-evolves and feeds back through cross terms --
        so the self-to-self sensitivity used everywhere downstream (c_own)
        reduces to a plain product of per-step scalars; no joint n*nx state,
        no multi-hop relay, no approximation relative to what was actually
        being consumed before (the cross-robot blocks of the old c_row were
        computed and then discarded -- see _compute_c_row in v29)."""
        p = self.p
        dt, a_ii, a_ij_gain, b_i = p['dt'], p['a_ii'], p['a_ij_gain'], p['b_i']
        nu, H = p['nu'], p['H']

        nbr_pos = {j: np.asarray(s, dtype=float)
                   for j, s in self._inbox.get('nbr_states', {}).items()}
        nbr_ids = sorted(nbr_pos.keys())

        u_i   = self._local_goal_dir()
        pos_h = self.pos.copy()
        a_seq  = []
        w_seq  = {j: [] for j in nbr_ids}   # per-step coupling gain dt*w_jh
        for _ in range(H):
            a_sum   = 0.0
            drift_h = np.zeros(2, dtype=float)
            for j in nbr_ids:
                p_j  = nbr_pos[j]
                d_ij = float(np.linalg.norm(pos_h - p_j)) + 1e-9
                w    = a_ij_gain / d_ij
                a_sum   += w
                drift_h += dt * w * p_j
                w_seq[j].append(dt * w)
            a_h = 1.0 + dt * a_ii - dt * a_sum
            a_seq.append(a_h)
            pos_h = a_h * pos_h + drift_h + dt * b_i * u_i

        e_i   = self._local_goal_dir_unit()
        c_own = np.zeros(H * nu, dtype=float)
        suffix = 1.0
        for h in range(H - 1, -1, -1):
            c_own[h*nu:(h+1)*nu] = suffix * dt * b_i * e_i
            suffix *= a_seq[h]

        # own_var = W_ii, the diagonal Gramian block: this robot's REAL,
        # un-normalised own-sensitivity magnitude. Previously c_own was
        # unit-normalised before its norm was read for this purpose, which
        # collapsed every robot's own_var to ~1 regardless of its actual
        # dynamics -- fixed here by reading the norm before normalising.
        raw_nrm      = float(np.linalg.norm(c_own))
        self.own_var = raw_nrm ** 2
        self.c_own   = c_own / raw_nrm if raw_nrm > 1e-6 else c_own

        # W_ij (off-diagonal block): the actual cross-sensitivity of this
        # robot's H-step rollout to neighbour j's position, i.e. the
        # accumulated Jacobian d(pos_H)/d(p_j) along the same suffix-of-a_h
        # chain used for c_own above -- derived from the real linearised
        # coupled dynamics (Section 4's f^(i) depending on x^(j)), not an
        # arbitrary or consensus-derived number. This replaces the old
        # a_i = ||c_own_i|| e_i construction, which had zero coupling terms
        # by definition.
        self._nbr_sens = {}
        for j in nbr_ids:
            s = 0.0
            suffix = 1.0
            for h in range(H - 1, -1, -1):
                s += suffix * w_seq[j][h]
                suffix *= a_seq[h]
            self._nbr_sens[j] = float(s)

    def _build_neighborhood_gramian(self):
        """Supervisor-directed fix: no global or consensus Gramian. Each
        agent assembles only its own local block -- the (1+|N(i)|)*nx-sized
        submatrix over {self} union {direct neighbours} -- fresh from this
        round's broadcast:
          * diagonal entries = each node's own W_ii (this agent's own_var,
            or the neighbour's own broadcast own_var) -- real per-agent
            dynamics, never averaged toward a shared value;
          * off-diagonal entries = the coupling block W_ij, built from the
            actual linearised-dynamics cross-sensitivity (_nbr_sens),
            symmetrised between this agent's own estimate i->j and the
            neighbour's own broadcast estimate j->i;
          * any pair with no direct edge (including neighbour-to-neighbour
            pairs this agent cannot observe) is left at zero, exactly as
            the block-Gramian construction specifies.
        This is a one-shot local read-and-assemble, not an iterative
        consensus fixed point -- rebuilt from scratch every planning step.
        """
        nx = self.p['nx']
        nbr_states   = self._inbox.get('nbr_states', {})
        nbr_own_var  = self._inbox.get('nbr_own_var', {})
        nbr_sens_map = self._inbox.get('nbr_sens_map', {})

        ids = [self.i] + sorted(nbr_states.keys())
        m   = len(ids)
        idx_of = {rid: k for k, rid in enumerate(ids)}

        # Genuine block matrix: (m*nx) x (m*nx), each block nx x nx.
        # Self is block-row/column 0, exactly Eq. (17)'s layout.
        W_sub = np.zeros((m * nx, m * nx), dtype=float)
        I_nx  = np.eye(nx)

        def blk(k_idx):
            return slice(k_idx * nx, (k_idx + 1) * nx)

        # Diagonal blocks: Wii = own_var * I_nx (Eq. 14, exact under the
        # isotropic surrogate dynamics -- see __init__ note).
        W_sub[blk(0), blk(0)] = self.own_var * I_nx
        for j in ids[1:]:
            k_idx = idx_of[j]
            W_sub[blk(k_idx), blk(k_idx)] = float(nbr_own_var.get(j, 0.0)) * I_nx

        # Off-diagonal blocks: Wij = 0.5*(Sij + Sji) * I_nx (Eq. 15-16),
        # only for direct edges; all other blocks (including
        # neighbour-to-neighbour pairs) stay exactly zero, matching
        # Eq. (17) and the "zero blocks mean not directly coupled"
        # requirement from Proof_of_set-2.pdf.
        for j in ids[1:]:
            k_idx = idx_of[j]
            s_ij = self._nbr_sens.get(j, 0.0)                     # my own estimate, i -> j
            s_ji = nbr_sens_map.get(j, {}).get(self.i, s_ij)      # neighbour j's own estimate, j -> i
            w_ij = 0.5 * (s_ij + s_ji)
            W_sub[blk(0), blk(k_idx)] = w_ij * I_nx
            W_sub[blk(k_idx), blk(0)] = w_ij * I_nx

        self._gramian_ids = ids
        self.W_local = W_sub

    def _local_eigendecompose(self):
        """Eigendecompose this agent's own (m*nx) x (m*nx) block Gramian
        W[i] = V[i] Lambda[i] (V[i])^T (Eq. 18) -- NOT a full-team or
        gossip-converged matrix, and NOT the old m x m scalar collapse.

        Then extract the ego block: the nx rows of V[i] that line up with
        this agent's own diagonal block in the Si ordering. Because
        _build_neighborhood_gramian always places self at block-row 0,
        that's simply rows 0:nx. This is V_i^[i] in the supervisor's
        notation -- Proof_of_set-2.pdf Step 6 -- an nx x k matrix whose k
        columns are this agent's own-coordinate components of the k
        retained joint eigenvectors. Nothing here is averaged across
        agents; it's a direct read of one agent's own row-block."""
        nx = self.p['nx']
        W  = self.W_local
        N  = W.shape[0]
        Ws = 0.5*(W + W.T) + 1e-6*np.eye(N)
        lam_cap = 20.0
        eigvals, eigvecs = np.linalg.eigh(Ws)
        idx = np.argsort(eigvals)[::-1]
        k = min(4, N)

        lam_k  = np.minimum(np.maximum(eigvals[idx][:k], 0.0), lam_cap)
        V_full = eigvecs[:, idx][:, :k]          # (N, k) joint eigenvectors
        if k < 4:
            lam_k  = np.pad(lam_k, (0, 4 - k))
            V_full = np.pad(V_full, ((0, 0), (0, 4 - k)))
        self.eigvals = lam_k

        # Ego row-block: self.i sits at block-row 0 by construction.
        self.eigvecs_ego = V_full[:nx, :]        # (nx, 4) -- V_i^[i]

    def _local_rollout_score(self, u_ref):
        """Kept as a standalone single-candidate diagnostic; action
        selection itself is now done by _mcts_search() below, which
        scores whole trees of multi-step sequences instead of one
        straight-line rollout per fixed candidate."""
        p  = self.p
        dt = p['dt']
        pos  = self.pos.copy()
        u    = np.asarray(u_ref, dtype=float)
        nm   = float(np.linalg.norm(u))
        u    = u/nm if nm > 1e-8 else self._local_goal_dir()
        R_base = p['r_robot'] + p['d_margin']
        total  = 0.0
        for _ in range(p['H']):
            pos    = pos + dt*u
            total -= float(np.linalg.norm(pos - self.safe_goal))
            for obs in self.obstacles:
                d = _obs_dist(obs, pos) - R_base
                if d < p['r_threat_obs']:
                    total -= 2.0*float(np.clip(p['r_threat_obs']-d, 0.0, p['r_threat_obs']))
        return total

    def _mcts_action_set(self):
        """The finite candidate-direction set every node in this round's
        tree branches over. The fan is built RELATIVE to the (already
        obstacle-aware) goal direction g -- not fixed world-frame angles --
        so a small simulation budget ell is spent on detours that could
        plausibly matter (mild-to-sharp side-steps around a neighbour or
        obstacle) rather than diluted across directions facing away from
        the goal. The SETS spectral candidates are appended unchanged."""
        g = self._local_goal_dir()
        actions = [g]
        n_half = max(1, int(self.p.get('mcts_n_angles', 6)) // 2)
        for off_deg in np.linspace(20.0, 150.0, n_half):
            th = np.radians(off_deg)
            for sign in (+1.0, -1.0):
                s = sign*th
                rot = np.array([[np.cos(s), -np.sin(s)],
                                 [np.sin(s),  np.cos(s)]])
                actions.append(rot @ g)
        # Eq. (19): Ai = {g_hat} u {+v_q^(i), -v_q^(i)}_{q=1..nbr} --
        # the candidate directions are the ego agent's own eigenvector
        # block, used directly as unit vectors, not scaled by sqrt(lam)
        # or re-projected through c_own. lam still ranks which modes are
        # worth branching on (n_branches of the top ones), exactly as
        # "leading eigenpairs" in the paper.
        n_ev = min(self.p['n_branches'], self.eigvecs_ego.shape[1])
        for mode in range(n_ev):
            v_q = self.eigvecs_ego[:, mode]
            nrm = float(np.linalg.norm(v_q))
            v_hat = v_q / nrm if nrm > 1e-8 else self._local_goal_dir_unit()
            for sign in (+1.0, -1.0):
                actions.append(sign * v_hat)
        return actions

    def _mcts_step(self, pos, a, rng):
        """One forward step of the local rollout model used for tree
        expansion and simulation. dt*noise is the only random element in
        the whole pipeline: it stands in for this robot's uncertainty
        about what its neighbours will actually do over the horizon --
        the 'MA' (multi-agent) term the convergence proof's
        kappa4^MA/sqrt(ell) bound refers to. Without it, every visit to a
        node would replay an identical deterministic trajectory and there
        would be nothing for repeated MCTS iterations to average over."""
        dt    = self.p['dt']
        sigma = float(self.p.get('mcts_sigma_ma', 0.05))
        noise = rng.normal(scale=sigma, size=2) if sigma > 0.0 else 0.0
        return pos + dt*a + dt*noise

    def _mcts_step_reward(self, pos):
        R_base = self.p['r_robot'] + self.p['d_margin']
        r = -float(np.linalg.norm(pos - self.safe_goal))
        for obs in self.obstacles:
            d = _obs_dist(obs, pos) - R_base
            if d < self.p['r_threat_obs']:
                r -= 2.0*float(np.clip(self.p['r_threat_obs'] - d,
                                       0.0, self.p['r_threat_obs']))
        return r

    def _ucb_select_child(self, node, c_ucb):
        """UCB1: argmax_a  V(child) + c * sqrt( ln T(node) / T(child) )."""
        logN = np.log(max(node.T, 1))
        best_a, best_score = None, -np.inf
        for a_idx, child in node.children.items():
            score = (np.inf if child.T == 0 else
                     child.V + c_ucb*np.sqrt(logN/child.T))
            if score > best_score:
                best_score, best_a = score, a_idx
        return best_a

    def _mcts_search(self):
        """The real select / expand / simulate / backpropagate loop, run
        entirely from this robot's own state, its own inbox-frozen
        neighbour positions, and its own obstacle list -- no different in
        kind from any other local-only method in this class. ell =
        self.p['K'] iterations grow one tree from a fresh root at the
        robot's current position; the action returned is the root child
        with the most visits (the standard 'robust child' rule)."""
        H     = int(self.p['H'])
        c_ucb = float(self.p.get('mcts_c_ucb', 1.0))
        ell   = max(1, int(self.p.get('K', 40)))
        rng   = np.random.default_rng()

        actions = self._mcts_action_set()
        n_a     = len(actions)
        if n_a == 0:
            return self._local_goal_dir()

        root = _MCTSNode(self.pos.copy(), depth=0, parent=None,
                          action=None, reward=0.0, untried=range(n_a))

        for _ in range(ell):
            node = root
            path = [node]

            # ---------- SELECTION ----------
            while (not node.untried) and node.children and node.depth < H:
                a_idx = self._ucb_select_child(node, c_ucb)
                node  = node.children[a_idx]
                path.append(node)

            # ---------- EXPANSION ----------
            if node.depth < H and node.untried:
                pick = node.untried.pop(int(rng.integers(len(node.untried))))
                new_pos = self._mcts_step(node.pos, actions[pick], rng)
                r       = self._mcts_step_reward(new_pos)
                nxt_untried = range(n_a) if node.depth + 1 < H else range(0)
                child = _MCTSNode(new_pos, node.depth+1, node, pick, r,
                                   untried=nxt_untried)
                node.children[pick] = child
                node = child
                path.append(node)

            # ---------- SIMULATION (straight-line rollout to H) ----------
            path_reward = sum(n.reward for n in path)
            roll_pos    = node.pos.copy()
            a_dir       = (actions[node.action] if node.action is not None
                           else self._local_goal_dir())
            tail = 0.0
            for _ in range(node.depth, H):
                roll_pos = self._mcts_step(roll_pos, a_dir, rng)
                tail    += self._mcts_step_reward(roll_pos)
            total_return = path_reward + tail

            # ---------- BACKPROPAGATION ----------
            for n in path:
                n.T += 1
                n.W += total_return
                n.V  = n.W / n.T

        if not root.children:
            return self._local_goal_dir()
        best_idx = max(root.children, key=lambda a: root.children[a].T)
        return actions[best_idx]

    def _select_best_branch(self):
        """Call site _plan() already uses; the decision is now made by a
        genuine MCTS search (T/V/UCB/expand/backprop, ell = self.p['K']
        iterations) over H-step action sequences, replacing the old
        one-ply greedy compare of a handful of fixed candidates."""
        return self._mcts_search()

    def _admm_step(self, u_nom):
        # Pre-QP EMA on u_nom only -- smooths MCTS's discrete branch choice
        # before it ever reaches the CBF/CLF constraints. The QP below still
        # solves fresh, every step, against the robot's real current position
        # and real current constraints -- this only changes what the QP is
        # aiming for, not what it's required to satisfy.
        u_nom_ema_alpha = 0.35
        if not hasattr(self, '_u_nom_smoothed'):
            self._u_nom_smoothed = u_nom.copy()
        else:
            self._u_nom_smoothed = (u_nom_ema_alpha * u_nom
                                     + (1.0 - u_nom_ema_alpha) * self._u_nom_smoothed)
        u_nom = self._u_nom_smoothed.copy()

        p = self.p
        nbr_states = self._inbox.get('nbr_states', {})
        nbr_drifts = self._inbox.get('nbr_drifts', {})

        nbr_ids  = sorted(nbr_states.keys())
        nbr_pos  = [np.asarray(nbr_states[j], dtype=float) for j in nbr_ids]
        nbr_drft = [np.asarray(nbr_drifts.get(j, np.zeros(2)), dtype=float)
                    for j in nbr_ids]

        nbr_z = self._inbox.get('nbr_z', {})
        z_received = {j: float(nbr_z.get(j, {}).get(self.i, 0.0)) for j in nbr_ids}

        f_i = self._local_drift()

        # Supervisor-directed: CLF weight ramp near goal. p_clf stays at
        # its baseline value everywhere except within a near-goal radius
        # (5x goal_tol), where it ramps linearly up to 5x baseline right
        # at the goal -- trusts the CLF goal-tracking term more (less
        # slack allowed) as the robot closes in, to damp final-approach
        # fluctuation. Untouched far from goal, as requested.
        dist_to_goal = float(np.linalg.norm(self.pos - self.goal))
        ramp_d0 = 2.5 * p['goal_tol']
        ramp_k = 2.1972245773362196  # width-based: 2*ln(9)/2.0 -- spreads transition over ~2 units instead of ~1
        ramp = 1.0 / (1.0 + np.exp(ramp_k * (dist_to_goal - ramp_d0)))
        p_clf_boost = 5.0
        p_clf_target = p['p_clf'] * (1.0 + (p_clf_boost - 1.0) * ramp)

        # EMA rate-limiter: caps how fast p_clf can actually change per step,
        # regardless of how steep the sigmoid is at this distance.
        p_clf_ema_alpha = 0.15
        if not hasattr(self, '_p_clf_smoothed'):
            self._p_clf_smoothed = p_clf_target
        else:
            self._p_clf_smoothed = (p_clf_ema_alpha * p_clf_target
                                     + (1.0 - p_clf_ema_alpha) * self._p_clf_smoothed)
        p_clf_eff = self._p_clf_smoothed

        u_safe, z_new = _solve_robot_qp_local(
            i            = self.i,
            p_i          = self.pos,
            u_nom        = u_nom,
            p_ref        = self.goal,
            nbr_positions= nbr_pos,
            nbr_drifts   = nbr_drft,
            nbr_ids      = nbr_ids,
            f_i          = f_i,
            lam_cbf_i    = self.lam_cbf,
            z_nbr_i      = z_received,
            obstacles    = self.obstacles,
            d_safe       = p['d_safe'],
            r_robot      = p['r_robot'],
            d_margin     = p['d_margin'],
            rc           = p['rc'],
            r_threat     = p['r_threat'],
            r_threat_obs = p['r_threat_obs'],
            gamma        = p['gamma'],
            gamma_clf    = p['gamma_clf'],
            p_clf        = p_clf_eff,
            epsilon      = p['epsilon'],
        )

        nm = float(np.linalg.norm(u_safe))
        if nm > 1.0:    u_safe = u_safe/nm
        elif nm < 1e-8: u_safe = u_nom.copy()

        for j, z_val in z_new.items():
            self.z_out[j] = float(z_val)

        for j in nbr_ids:
            a_ij = self.pos - np.asarray(nbr_states[j], dtype=float)
            dist = float(np.linalg.norm(a_ij)) + 1e-9
            if dist > p['r_threat']: continue
            h_ij  = dist**2 - p['d_safe']**2
            D_ij  = float(np.clip(2.0*np.dot(a_ij, f_i - nbr_drft[nbr_ids.index(j)]),
                                  -2.0, 2.0))
            z_i   = self.z_out.get(j, 0.0)
            z_j   = z_received.get(j, 0.0)
            g_val = (-2.0*float(np.dot(a_ij, u_safe))
                     - p['gamma']*h_ij - D_ij + z_i - z_j)
            lam_old = self.lam_cbf.get(j, 0.0)
            lam_new = lam_old*0.90 if g_val < 0.0 else lam_old + p['rho_lam']*g_val
            self.lam_cbf[j] = float(np.clip(lam_new, 0.0, p['lam_max']))

        return u_safe

    def _local_drift(self) -> np.ndarray:
        p = self.p
        nbr_states = self._inbox.get('nbr_states', {})
        fi = np.zeros(2, dtype=float)
        for j, p_j in nbr_states.items():
            p_j  = np.asarray(p_j, dtype=float)
            diff = p_j - self.pos
            d    = float(np.linalg.norm(diff))
            if 1e-8 < d:
                fi += p['coupling'] * diff / d
        return fi

    def _local_goal_dir(self) -> np.ndarray:
        p_i   = self.pos
        goal  = self.safe_goal
        diff  = goal - p_i
        seg   = float(np.linalg.norm(diff))
        if seg < self.p['goal_tol']:
            return np.zeros(self.p['nu'], dtype=float)
        e_dir  = diff/seg
        R_base = self.p['r_robot'] + self.p['d_margin']
        gamma  = self.p['gamma']
        r_to   = self.p['r_threat_obs']

        worst_viol = 0.0; worst_c = None
        for obs in self.obstacles:
            for (c_ls, r_ls) in _probe_circles(obs, rc=self.p['rc']):
                c_ls  = np.asarray(c_ls, float)
                R_eff = float(r_ls) + R_base
                a_im  = p_i - c_ls
                d_obs = float(np.linalg.norm(a_im))
                if d_obs - R_eff > r_to: continue
                h    = d_obs**2 - R_eff**2
                viol = -gamma*h - float(np.dot(2.0*a_im, e_dir))
                if viol > worst_viol:
                    worst_viol = viol; worst_c = c_ls

        R_park = self.p['r_robot'] + 0.5 * self.p['d_safe']
        nbr_states  = self._inbox.get('nbr_states', {})
        nbr_arrived = self._inbox.get('nbr_arrived', {})
        for j, p_j in nbr_states.items():
            if not nbr_arrived.get(j, False): continue
            p_j  = np.asarray(p_j, float)
            a_im = p_i - p_j
            d_obs = float(np.linalg.norm(a_im))
            if d_obs - R_park > r_to: continue
            h    = d_obs**2 - R_park**2
            viol = -gamma*h - float(np.dot(2.0*a_im, e_dir))
            if viol > worst_viol:
                worst_viol = viol; worst_c = p_j

        if worst_c is None:
            return e_dir

        a     = p_i - worst_c
        n_hat = a/(float(np.linalg.norm(a)) + 1e-9)
        tang1 = np.array([-n_hat[1],  n_hat[0]])
        tang2 = np.array([ n_hat[1], -n_hat[0]])
        goal_vec = goal - worst_c
        e_tang = tang1 if np.dot(tang1, goal_vec) >= np.dot(tang2, goal_vec) else tang2
        if float(np.dot(e_tang, e_dir)) < -0.3:
            e_tang = -e_tang
        alpha = float(np.clip(worst_viol/(worst_viol+1.0), 0.3, 0.9))
        u = (1.0-alpha)*e_dir + alpha*e_tang
        nm = float(np.linalg.norm(u))
        return u/nm if nm > 1e-8 else e_tang

    def _local_goal_dir_unit(self) -> np.ndarray:
        diff = self.safe_goal - self.pos
        d = float(np.linalg.norm(diff))
        return diff/d if d > 1e-8 else np.array([1.0, 0.0])

    def _update_stall(self):
        if self._prev_pos is not None:
            moved = float(np.linalg.norm(self.pos - self._prev_pos))
            if moved < self._STALL_THRESH:
                self._stall_count += 1
            else:
                self._stall_count = 0
        self._prev_pos = self.pos.copy()

    def _closest_obstacle_info(self):
        best_d = np.inf
        best_n = np.array([1.0, 0.0])
        p = self.pos
        for obs in self.obstacles:
            d = _obs_dist(obs, p)
            if d < best_d:
                best_d = d
                best_n = _obs_normal(obs, p)
        return best_d, best_n

    def _update_progress(self):
        dist = float(np.linalg.norm(self.pos - self.goal))
        if self._prev_goal_dist is not None and dist >= self._prev_goal_dist - _NO_PROGRESS_STEP:
            self._no_progress += 1
        else:
            self._no_progress = 0
        self._prev_goal_dist = dist

    def _repulsive_field(self) -> np.ndarray:
        p = self.pos
        rep = np.zeros(2, dtype=float)
        R_safe = self.p['r_robot'] + self.p['d_margin'] + self.p['rc']
        for obs in self.obstacles:
            d = float(_obs_dist(obs, p)) - R_safe
            if d >= _REPULSE_R_OBS:
                continue
            d_eff = max(d, 0.02)
            n_out = _obs_normal(obs, p)
            w = _REPULSE_GAIN_OBS * (1.0 / d_eff - 1.0 / _REPULSE_R_OBS) / (d_eff * d_eff)
            rep += w * n_out

        nbr_states = self._inbox.get('nbr_states', {})
        for j, p_j in nbr_states.items():
            diff = p - np.asarray(p_j, dtype=float)
            dist = float(np.linalg.norm(diff)) + 1e-9
            if dist >= _REPULSE_R_ROB:
                continue
            d_eff = max(dist - self.p['d_safe'], 0.03)
            w = _REPULSE_GAIN_ROB * (1.0 / d_eff - 1.0 / _REPULSE_R_ROB) / (d_eff * d_eff)
            rep += w * (diff / dist)
        return rep

    def _orbit_tangent_toward_goal(self, g_hat: np.ndarray) -> np.ndarray:
        _, normal = self._closest_obstacle_info()
        tang1 = np.array([-normal[1], normal[0]], dtype=float)
        tang2 = -tang1
        t1n = float(np.linalg.norm(tang1))
        if t1n < 1e-8:
            return g_hat
        tang1 /= t1n
        tang2 /= float(np.linalg.norm(tang2))
        return tang1 if float(tang1 @ g_hat) >= float(tang2 @ g_hat) else tang2

    def _jump_magnitude(self) -> float:
        extra = max(0, self._stall_count - self._STALL_STEPS)
        jump = min(_JUMP_MAX, _OUTWARD_STEP + _JUMP_PER_STALL * extra)
        if self._no_progress >= 6:
            jump = min(1.0, jump + 0.20)
        return jump

    def _needs_escape(self, u: np.ndarray) -> bool:
        if self.arrived:
            return False
        frozen = float(np.linalg.norm(u)) < _STALL_ACTION_EPS
        stalled = self._stall_count >= self._STALL_STEPS
        return frozen or stalled or self._no_progress >= 8

    def _stall_escape_dir(self) -> np.ndarray:
        g_hat = self._local_goal_dir_unit()
        u_nom = self._local_goal_dir()
        f_i   = self._local_drift()
        rep   = self._repulsive_field()
        rep_n = float(np.linalg.norm(rep))

        if rep_n > 1e-8:
            blend = 0.70 * g_hat + 0.30 * (rep / rep_n)
        else:
            blend = u_nom
        bn = float(np.linalg.norm(blend))
        u_nom2 = blend / bn if bn > 1e-8 else g_hat

        u_esc = _project_onto_obs_cbf(
            u_nom2, self.pos, f_i, self.obstacles,
            _R_ROBOT_ESCAPE,
            self.p['rc'],
            self.p['d_margin'] * 0.5,
            self.p['gamma'],
            self.p['r_threat_obs'],
        )

        jump = self._jump_magnitude()
        if float(np.linalg.norm(u_esc)) >= _STALL_ACTION_EPS and self._no_progress < 8:
            un = float(np.linalg.norm(u_esc))
            if un > jump:
                u_esc = u_esc / un * jump
            self._prev_escape = u_esc.copy()
            return u_esc

        orbit = self._orbit_tangent_toward_goal(g_hat)
        if rep_n > 1e-8:
            burst = 0.30 * (rep / rep_n) + 0.35 * orbit + 0.35 * g_hat
        else:
            _, normal = self._closest_obstacle_info()
            burst = 0.35 * normal + 0.35 * orbit + 0.30 * g_hat

        prev = self._prev_escape
        if prev is not None:
            pn = float(np.linalg.norm(prev))
            if pn > 1e-8 and float(burst @ prev) < -0.15 * pn * float(np.linalg.norm(burst)):
                perp = np.array([-burst[1], burst[0]], dtype=float)
                burst = burst + (0.55 * g_hat if float(perp @ g_hat) >= 0 else -0.55 * perp)

        bn = float(np.linalg.norm(burst))
        action = (burst / bn) * jump if bn > 1e-8 else g_hat * jump
        self._prev_escape = action.copy()
        return action

    def finalize_action(self, u_nom: np.ndarray) -> np.ndarray:
        """The last decision in the pipeline -- obstacle-CBF safety
        projection plus stall escape -- using only this robot's own
        position, its own a-priori obstacle list, and its own drift.
        Nothing here ever touched env or another agent's internals."""
        f_i = self._local_drift()
        u = _project_onto_obs_cbf(
            u_nom, self.pos, f_i, self.obstacles,
            self.p['r_robot'], self.p['rc'], self.p['d_margin'],
            self.p['gamma'], self.p['r_threat_obs'])
        nm = float(np.linalg.norm(u))
        if nm > 1.0:
            u = u / nm
        elif nm < 1e-8:
            u = self._local_goal_dir()
        if self._needs_escape(u):
            u = self._stall_escape_dir()
        self.action = u
        return u


class MessageRouter:
    """A wireless channel model, nothing more. Every transmission reaches
    every robot (free-space broadcast); each robot decides for itself,
    inside receive(), which of those packets are close enough to count as
    a neighbour. The router never reads an agent's attributes directly --
    only get_outbox(), the agent's own explicit publish call."""

    def __init__(self, agents: list[RobotAgentFullyDecent], sensing_radius: float):
        self.agents = agents
        self.radius = sensing_radius

    def broadcast(self):
        outboxes = [a.get_outbox() for a in self.agents]
        for agent in self.agents:
            agent.receive(outboxes, self.radius)


class SETSPlannerFullyDecent:
    """A clock, not a decision-maker. Every domain quantity -- arrival,
    linearisation, Gramian, gossip weights, ADMM step, CBF projection,
    escape -- is computed by the agent that owns it, from its own state and
    its own inbox. This class only: (1) hands each robot its own sensed
    position (the one thing env legitimately provides, analogous to
    GPS/odometry), (2) tells the router when to run a broadcast round, and
    (3) collects the resulting actions for the simulator. It performs no
    physics, no global linearisation, and no global-knowledge computation
    of its own (eps is gone; gossip weights are now computed locally inside
    each agent from purely local degree information)."""

    def __init__(self, env: EnvSETSv5, H=5, K=60,
                 T_gossip=5, K_admm=5, n_branches=2,
                 mcts_n_angles=6, mcts_c_ucb=1.0, mcts_sigma_ma=0.02):
        self.env      = env
        self.T_gossip = T_gossip
        self.K_admm   = K_admm
        n  = env.n_robots()
        nu = env.nu

        params = dict(
            H            = H,
            # K doubles as ell, the MCTS iteration count: each agent grows
            # its own tree by K simulations per planning call (see
            # RobotAgentFullyDecent._mcts_search). Previously stored here
            # and never read by anything -- now the thing Theorem 6's
            # kappa4^MA/sqrt(ell) term actually refers to.
            K            = K,
            mcts_n_angles= mcts_n_angles,
            mcts_c_ucb   = mcts_c_ucb,
            mcts_sigma_ma= mcts_sigma_ma,
            nx           = env.nx,
            nu           = nu,
            dt           = getattr(env, 'dt', 0.1),
            goal_tol     = env.goal_tol,
            coupling     = getattr(env, 'coupling', 0.0),
            # surrogate linear model used only for the Gramian / spectral
            # branch selection -- a priori known constants, same role as
            # d_safe/r_robot below, not a runtime query of env.
            a_ii         = getattr(env, 'a_ii', 0.2),
            a_ij_gain    = getattr(env, 'a_ij_gain', 0.2),
            b_i          = getattr(env, 'b_i', 1.0),
            n_branches   = n_branches,
            d_safe       = 0.22,
            r_robot      = 0.15,
            d_margin     = 0.10,
            rc           = 0.12,
            r_threat     = 1.5,
            r_threat_obs = 1.5,
            gamma        = 3.0,
            gamma_clf    = 1.0,
            p_clf        = 5.0,
            epsilon      = 0.001,
            rho_lam      = 0.1,
            lam_max      = 3.0,
        )
        self.d_safe = params['d_safe']

        self.agents = [
            RobotAgentFullyDecent(
                robot_id  = i,
                n_robots  = n,
                goal_pos  = env.landmarks[env.robots[i].target].pos,
                obstacles = env.obstacles,
                params    = params,
            )
            for i in range(n)
        ]

        self.router = MessageRouter(self.agents, sensing_radius=env.radius)
        # diagnostic only -- computed after the fact from agents' own
        # locally-known degree, never fed back into any agent decision.
        self._eps_gossip_current = 0.0

    def plan(self):
        env    = self.env
        agents = self.agents

        # Sensing: each robot reads its own measured position from the
        # physical world. This is the only env touch in the whole pipeline,
        # and it never crosses between robots.
        for i, agent in enumerate(agents):
            agent.sense_self(env.robots[i].pos)

        self.router.broadcast()

        for agent in agents:
            agent._update_stall()
            if not agent.arrived:
                agent._update_progress()
            if agent.arrived:
                continue
            agent._local_linearise()

        # Supervisor-directed fix: no gossip/consensus rounds. Each agent's
        # own_var and _nbr_sens (computed above from its own local
        # linearisation) are broadcast once; every agent then assembles its
        # own neighbourhood Gramian submatrix directly from this round's
        # inbox -- a one-shot local read, not an iterative fixed point.
        # T_gossip is kept as a constructor argument for interface
        # compatibility with existing call sites but no longer drives a
        # loop here.
        self.router.broadcast()
        for agent in agents:
            if not agent.arrived:
                agent._build_neighborhood_gramian()

        u_nom_list = []
        for agent in agents:
            if agent.arrived:
                u_nom_list.append(np.zeros(env.nu))
            else:
                agent._local_eigendecompose()
                u_nom_list.append(agent._select_best_branch())

        self.router.broadcast()
        for _ in range(self.K_admm):
            u_new = []
            for i, agent in enumerate(agents):
                if agent.arrived:
                    u_new.append(np.zeros(env.nu))
                else:
                    u_new.append(agent._admm_step(u_nom_list[i]))
            u_nom_list = u_new
            self.router.broadcast()

        actions = []
        for i, agent in enumerate(agents):
            if agent.arrived:
                agent.action = np.zeros(env.nu)
                actions.append(agent.action)
            else:
                actions.append(agent.finalize_action(u_nom_list[i]))

        degrees = [len(a._inbox.get('nbr_states', {})) for a in agents]
        self._eps_gossip_current = float(np.mean(degrees)) if degrees else 0.0

        eigvals_all = [a.eigvals.copy() for a in agents]
        eigvecs_all = [a.eigvecs_ego.copy() for a in agents]

        return actions, eigvals_all, eigvecs_all
