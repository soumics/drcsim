#!/usr/bin/env python3
#
# Copyright 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Torque-level whole-body controller for Atlas (Pinocchio + ProxQP), no rclpy.

One QP per tick over joint accelerations and the two foot contact wrenches:

    minimise   sum_k w_k |A_k x - b_k|^2        x = [qdd (36), f_l (6), f_r (6)]
    subject to floating-base dynamics            M_b qdd + h_b = J_b^T f
               feet don't move                   J_c qdd + dJ_c v = 0
               unilateral, friction, CoP in sole (per foot, sole frame)
               joint torque limits                tau = M_j qdd + h_j - J_j^T f

Tasks: CoM acceleration (PD toward a target, set by the caller -- balance
or a capture-point law), pelvis orientation, joint posture. The joint
torques tau go to AtlasPlugin as effort with kp = 0 (see TorqueCommand).

State: joint angles/velocities from AtlasState, pelvis orientation and
angular velocity from its IMU; the pelvis position and linear velocity
from the feet, which are assumed not to move (both on the ground).
"""

import numpy as np
import pinocchio as pin
import proxsuite

SOLE = (0.023, 0.0, -0.0741)  # sole centre from the ankle (foot frame), atlas_v5 URDF
# Palm centre in the SCHUNK SVH base frame (fingers +z, palm faces +y, thumb +x).
SVH_PALM = (0.0, 0.03, 0.09)
HAND_ERR_MAX = 0.08   # m: hand task position error saturation
MAX_ITER = 200        # ProxQP iterations per solve
LIMIT_HORIZON = 0.15  # s: joints must be able to stop within this before a limit
LIMIT_MARGIN = 0.02   # rad
SOLE_HALF = (0.100, 0.060)    # usable CoP extent behind the sole centre / half-width
SOLE_FRONT = 0.130            # usable CoP extent ahead of it (sole to 0.114, toe pad to 0.15)
MU = 0.6
FZ_MIN = 20.0


class WholeBodyController:
    """
    Build from a URDF string and the actuated joint names (AtlasCommand order).

    Other joints in the URDF (fingers, lidar spindle) are locked at zero.
    """

    def __init__(self, urdf, joint_names):
        full = pin.buildModelFromXML(urdf, pin.JointModelFreeFlyer())
        lock = [full.getJointId(n) for n in full.names[1:]
                if n not in joint_names and n != 'root_joint']
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        m = self.model
        self.names = list(joint_names)
        self.iq = np.array([m.joints[m.getJointId(n)].idx_q for n in joint_names])
        self.iv = np.array([m.joints[m.getJointId(n)].idx_v for n in joint_names])
        self.feet = {}
        for side in 'lr':
            foot = m.getFrameId(f'{side}_foot')
            frame = m.frames[foot]
            placement = frame.placement * pin.SE3(np.eye(3), np.array(SOLE))
            self.feet[side] = m.addFrame(pin.Frame(
                f'{side}_sole', frame.parentJoint, foot, placement, pin.FrameType.OP_FRAME))
        # Palm frames, when the model has SVH hands (manipulation tasks).
        self.palms = {}
        for side in 'lr':
            if m.existFrame(f'{side}_svh_base_link'):
                base = m.getFrameId(f'{side}_svh_base_link')
                frame = m.frames[base]
                self.palms[side] = m.addFrame(pin.Frame(
                    f'{side}_palm', frame.parentJoint, base,
                    frame.placement * pin.SE3(np.eye(3), np.array(SVH_PALM)),
                    pin.FrameType.OP_FRAME))
        self.data = m.createData()
        self.mass = pin.computeTotalMass(m)
        self.tau_max = m.effortLimit[self.iv]
        self.sole_world = None  # sole positions, fixed while standing
        self.ground = {}         # where a lifted foot left the ground (world)
        self.last = {}

    # --- state -------------------------------------------------------

    def state(self, position, velocity, quat_xyzw, omega_body, contacts='lr'):
        """
        Return (q, v) of the floating-base model, the pelvis placed from the feet.

        Only feet in `contacts` (loaded, per the foot sensors) are trusted
        to be still; a lifted foot's remembered position is updated.
        """
        m, d = self.model, self.data
        q = pin.neutral(m)
        q[self.iq] = position
        q[3:7] = quat_xyzw
        v = np.zeros(m.nv)
        v[self.iv] = velocity
        v[3:6] = omega_body
        # Pelvis position: the soles sit where they were first seen.
        pin.framesForwardKinematics(m, d, q)
        soles = {s: d.oMf[f].translation.copy() for s, f in self.feet.items()}
        if self.sole_world is None:
            self.sole_world = soles
        contacts = contacts or 'lr'
        q[:3] = np.mean([self.sole_world[s] - soles[s] for s in contacts], axis=0)
        for s in 'lr':
            if s not in contacts:
                self.ground.setdefault(s, self.sole_world[s].copy())
                self.sole_world[s] = q[:3] + soles[s]
            else:
                self.ground.pop(s, None)
        # Pelvis linear velocity: the soles don't move.
        pin.computeJointJacobians(m, d, q)
        pin.updateFramePlacements(m, d)
        rows, rhs = [], []
        for f in (self.feet[s] for s in contacts):
            J = pin.getFrameJacobian(m, d, f, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]
            rows.append(J[:, :3])
            rhs.append(-(J[:, 3:] @ v[3:]))
        lin_world = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)[0]
        # The free-flyer velocity is in the pelvis frame; J[:, :3] above maps
        # it to the world, so this is already the local linear velocity.
        v[:3] = lin_world
        return q, v

    # --- control -----------------------------------------------------

    def solve(self, q, v, com_acc, rot_des=np.eye(3), posture=None, contacts='lr',
              kp_rot=100.0, kd_rot=20.0, kp_post=60.0, kd_post=15.0,
              w_com=100.0, w_rot=30.0, w_post=1.0, foot_targets=None,
              kp_foot=200.0, kd_foot=28.0, w_foot=300.0, hand_targets=None,
              kp_hand=150.0, kd_hand=25.0, w_hand=200.0, squeeze=0.0):
        """
        Return the joint torques (AtlasCommand order) for the tasks.

        com_acc: desired CoM acceleration (world, 3); rot_des: desired pelvis
        orientation (world); posture: desired joint angles (default: now);
        contacts: the feet on the ground -- a lifted foot gets no wrench and
        no no-slip constraint, but a task bringing it level to
        foot_targets[side] (world sole position, or (position, yaw[, pitch]);
        default: where it left the
        ground). Without that task the CoM task used the free leg's mass and
        held it up, and Atlas toppled off the other foot (measured).
        hand_targets: {side: (palm position, palm rotation)} in the world --
        a 6D task per palm (SVH hands). Pushing a target into an object makes
        the hand press on it (about kp_hand x depth x the arm's mass).
        squeeze: N each palm presses along its normal on an object held between
        them -- modelled as the object pushing back on the palms, so the torques
        produce the squeeze and the balance accounts for it. (Squeeze torques
        added after the solve tilted Atlas forward, 20 deg at 200 N: measured.)
        """
        m, d = self.model, self.data
        nv = m.nv
        pin.computeAllTerms(m, d, q, v)
        pin.updateFramePlacements(m, d)
        M, h = d.M, d.nle
        pin.forwardKinematics(m, d, q, v, np.zeros(nv))
        pin.updateFramePlacements(m, d)
        Jc, drift = [], []
        for f in self.feet.values():
            Jc.append(pin.getFrameJacobian(m, d, f, pin.ReferenceFrame.LOCAL))
            drift.append(pin.getFrameAcceleration(m, d, f, pin.ReferenceFrame.LOCAL).vector)
        Jc = np.vstack(Jc)                  # 12 x nv
        drift = np.concatenate(drift)       # 12
        com = pin.centerOfMass(m, d, q, v, np.zeros(nv))
        com_drift = d.acom[0].copy()
        Jcom = pin.jacobianCenterOfMass(m, d, q)
        n = nv + 12

        def task(A_qdd, b):
            A = np.zeros((A_qdd.shape[0], n))
            A[:, :nv] = A_qdd
            return A, b

        tasks = []
        tasks.append((w_com,) + task(Jcom, com_acc - com_drift))
        R = d.oMi[1].rotation
        err = pin.log3(R.T @ rot_des)
        sel = np.zeros((3, nv))
        sel[:, 3:6] = np.eye(3)
        tasks.append((w_rot,) + task(sel, kp_rot * err - kd_rot * v[3:6]))
        qj = q[self.iq]
        post = qj if posture is None else np.asarray(posture)
        selj = np.zeros((len(self.iv), nv))
        selj[np.arange(len(self.iv)), self.iv] = 1.0
        tasks.append((w_post,) + task(selj, kp_post * (post - qj) - kd_post * v[self.iv]))
        for side, f in self.feet.items():
            if side in contacts:
                continue
            target = (foot_targets or {}).get(side, self.ground.get(side))
            if target is None:
                continue
            Jf = pin.getFrameJacobian(m, d, f, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            vf = Jf @ v
            af = pin.getFrameAcceleration(m, d, f, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Rf = d.oMf[f].rotation
            yaw = np.arctan2(Rf[1, 0], Rf[0, 0])
            pitch = 0.0
            if isinstance(target, tuple):  # (position, yaw[, pitch]): a walking step
                target, yaw, *rest = target
                pitch = rest[0] if rest else 0.0  # > 0: toe down, heel up
            level = pin.rpy.rpyToMatrix(0.0, pitch, yaw)
            want = np.concatenate([
                kp_foot * (np.asarray(target) - d.oMf[f].translation) - kd_foot * vf[:3],
                kp_foot * (Rf @ pin.log3(Rf.T @ level)) - kd_foot * vf[3:]])
            tasks.append((w_foot,) + task(Jf, want - af.vector))
        for side, (target_pos, target_rot) in (hand_targets or {}).items():
            f = self.palms[side]
            Jh = pin.getFrameJacobian(m, d, f, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            vh = Jh @ v
            ah = pin.getFrameAcceleration(m, d, f, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Rh = d.oMf[f].rotation
            # Saturated: a palm far behind its target asked for a huge
            # acceleration and the arm dragged the body over (measured).
            err = np.asarray(target_pos) - d.oMf[f].translation
            err *= min(1.0, HAND_ERR_MAX / max(np.linalg.norm(err), 1e-9))
            want = np.concatenate([
                kp_hand * err - kd_hand * vh[:3],
                kp_hand * (Rh @ pin.log3(Rh.T @ np.asarray(target_rot))) - kd_hand * vh[3:]])
            tasks.append((w_hand,) + task(Jh, want - ah.vector))
        if squeeze > 0.0:
            ext = np.zeros(nv)
            for f in self.palms.values():
                Jp = pin.getFrameJacobian(m, d, f, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]
                ext += Jp.T @ (-squeeze * d.oMf[f].rotation[:, 1])
            h = h - ext  # M qdd + h = S tau + Jc^T f + ext
        H = 1e-6 * np.eye(n)
        H[nv:, nv:] += 1e-5 * np.eye(12)
        g = np.zeros(n)
        for w, A, b in tasks:
            H += w * A.T @ A
            g -= w * A.T @ b

        # Equalities: base dynamics (6), then per foot either no slip (in
        # contact) or zero wrench (lifted).
        A_eq = np.zeros((18, n))
        b_eq = np.zeros(18)
        A_eq[:6, :nv] = M[:6]
        A_eq[:6, nv:] = -Jc[:, :6].T
        b_eq[:6] = -h[:6]
        for k, side in enumerate('lr'):
            rows = slice(6 + 6 * k, 12 + 6 * k)
            if side in contacts:
                A_eq[rows, :nv] = Jc[6 * k:6 * k + 6]
                b_eq[rows] = -drift[6 * k:6 * k + 6]
            else:
                A_eq[rows, nv + 6 * k:nv + 6 * k + 6] = np.eye(6)
        # Inequalities: contact wrench cone per foot, torque limits.
        L, W = SOLE_HALF
        cone = np.array([
            [0, 0, 1, 0, 0, 0],
            [1, 0, -MU, 0, 0, 0], [-1, 0, -MU, 0, 0, 0],
            [0, 1, -MU, 0, 0, 0], [0, -1, -MU, 0, 0, 0],
            [0, 0, -W, 1, 0, 0], [0, 0, -W, -1, 0, 0],
            # CoP x = -m_y / f_z within [-L, SOLE_FRONT].
            [0, 0, -L, 0, 1, 0], [0, 0, -SOLE_FRONT, 0, -1, 0],
            [0, 0, -MU * (L + W), 0, 0, 1], [0, 0, -MU * (L + W), 0, 0, -1],
        ])
        nc = cone.shape[0]
        nj = len(self.iv)
        C = np.zeros((2 * nc + 2 * nj, n))
        lo = np.full(2 * nc + 2 * nj, -1e20)
        hi = np.zeros(2 * nc + 2 * nj)
        for k, side in enumerate('lr'):
            C[k * nc:(k + 1) * nc, nv + 6 * k:nv + 6 * k + 6] = cone
            lo[k * nc] = FZ_MIN if side in contacts else -1e20
            hi[k * nc] = 1e20
        # tau = M_j qdd + h_j - J_j^T f
        C[2 * nc:2 * nc + nj, :nv] = M[self.iv]
        C[2 * nc:2 * nc + nj, nv:] = -Jc[:, self.iv].T
        lo[2 * nc:2 * nc + nj] = -self.tau_max - h[self.iv]
        hi[2 * nc:2 * nc + nj] = self.tau_max - h[self.iv]
        # Joint limits as acceleration bounds: stop within LIMIT_HORIZON. The
        # QP didn't know the elbow can't straighten past 0 and reached for a
        # box by hyperextending it instead of leaning (measured).
        qj, vj = q[self.iq], v[self.iv]
        T = LIMIT_HORIZON
        q_lo = m.lowerPositionLimit[self.iq] + LIMIT_MARGIN
        q_hi = m.upperPositionLimit[self.iq] - LIMIT_MARGIN
        a_lo = 2 * (q_lo - qj - vj * T) / T ** 2
        a_hi = 2 * (q_hi - qj - vj * T) / T ** 2
        bad = a_lo > a_hi  # already past a limit: just brake
        a_lo[bad] = a_hi[bad] = -vj[bad] / T
        rows = slice(2 * nc + nj, 2 * nc + 2 * nj)
        C[rows, :nv] = selj
        lo[rows], hi[rows] = np.clip(a_lo, -1e20, 1e20), np.clip(a_hi, -1e20, 1e20)

        qp = proxsuite.proxqp.dense.QP(n, 18, C.shape[0])
        qp.settings.eps_abs = 1e-6
        qp.settings.verbose = False
        # Capped: ProxQP's default limits let one hard (nearly infeasible)
        # problem run for ~2 minutes, and with lockstep the simulation went
        # on without commands until Atlas fell (measured). A solve normally
        # takes a few dozen iterations.
        qp.settings.max_iter = MAX_ITER
        qp.settings.max_iter_in = MAX_ITER
        qp.init(H, g, A_eq, b_eq, C, lo, hi)
        qp.solve()
        status = qp.results.info.status
        if status != proxsuite.proxqp.QPSolverOutput.PROXQP_SOLVED and 'tau' in self.last:
            # Not solved this tick: keep the last good torques (1 ms old).
            self.last.update(status=status, iter=qp.results.info.iter,
                             failures=self.last.get('failures', 0) + 1)
            return self.last['tau']
        x = qp.results.x
        qdd, f = x[:nv], x[nv:]
        tau = M[self.iv] @ qdd + h[self.iv] - Jc[:, self.iv].T @ f
        self.last = {'com': com, 'f': f, 'qdd': qdd, 'status': status,
                     'iter': qp.results.info.iter, 'tau': tau,
                     'failures': self.last.get('failures', 0)}
        return tau

    def set_payload(self, mass, reach):
        """
        Add (or with mass 0, remove) a held object, split between the palms.

        Half the mass sits reach m in front of each palm (along its +y
        normal), so the CoM, gravity and dynamics include it: without it a
        10 kg box held out in front pulled Atlas over (measured).
        """
        m = self.model
        if not hasattr(self, '_bare_inertia'):
            self._bare_inertia = {s: m.inertias[m.frames[f].parentJoint].copy()
                                  for s, f in self.palms.items()}
        for side, f in self.palms.items():
            frame = m.frames[f]
            inertia = self._bare_inertia[side].copy()
            if mass > 0:
                at = frame.placement.act(np.array([0.0, reach, 0.0]))
                inertia += pin.Inertia(mass / 2, at, np.zeros((3, 3)))
            m.inertias[frame.parentJoint] = inertia
        self.mass = pin.computeTotalMass(m)

    def com_velocity(self, q, v):
        """Return the CoM velocity (world)."""
        pin.centerOfMass(self.model, self.data, q, v)
        return self.data.vcom[0].copy()

    def sole_positions(self, q):
        pin.framesForwardKinematics(self.model, self.data, q)
        return {s: self.data.oMf[f].translation.copy() for s, f in self.feet.items()}
