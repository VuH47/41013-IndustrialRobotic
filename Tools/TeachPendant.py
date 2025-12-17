"""
Example
-------
    from MinhTool import TeachPendant
    pendant = TeachPendant().add_to_env(env, robot)
    while True:
        pendant.tick()
        env.step()
"""

from __future__ import annotations
import numpy as np
import swift
from spatialmath import SE3


class TeachPendant:
    """
    Provides joint sliders, jog buttons, Cartesian jog, tool/base frame toggle,
    and a simple "record + playback".
    """

    def __init__(self, label: str = "Teach Pendant"):
        self.label = label
        self.env = None
        self.robot = None

       
        self._sliders = []  # joint sliders
        self._widgets = []  # jog buttons, step sliders, frame toggle

        # Jog settings
        self.joint_step_deg = 1.0  # deg per joint jog
        self.xyz_step_m = 0.01  # 1 cm per XYZ jog
        self.rpy_step_deg = 5.0  # deg per rotational jog
        self.frame = "tool"  # tool and base
        self._frame_btn = None
        self._sel_joint = 0  # currently selected joint for jog buttons
        self._recording = False
        self._recorded_traj: list[np.ndarray] = []
        self._playback = False
        self._playback_index = 0
        self._look_btn = None

    # Public to call ---------------------------------------------------------
    def add_to_env(self, env, robot):
        self.env = env
        self.robot = robot
        self._build_ui()
        return self

    def remove(self):
        if not self.env:
            return
        for w in self._sliders + self._widgets:
            try:
                self.env.remove(w)
            except Exception:
                pass
        self._sliders, self._widgets = [], []
        self._frame_btn = None
        self._look_btn = None
        self._recording = False
        self._playback = False
        self._recorded_traj = []
        self._playback_index = 0

    # UI--------------------------------------------------------
    def _build_ui(self):
        self.remove()
        self._build_joint_sliders_and_jogs()
        self._build_cartesian_jogs()

    def _build_joint_sliders_and_jogs(self):
        name = getattr(self.robot, "name", "Robot")
        for j, link in enumerate(getattr(self.robot, "links", [])):
            if getattr(link, "isjoint", False):
                qlim = getattr(link, "qlim", None)
                if qlim is None:
                    qmin, qmax = -np.pi, np.pi
                else:
                    qmin, qmax = float(qlim[0]), float(qlim[1])

                # Joint slider in degrees
                slider = swift.Slider(
                    lambda x, j=j: self._on_joint_slider(j, x),
                    min=float(np.round(np.rad2deg(qmin), 2)),
                    max=float(np.round(np.rad2deg(qmax), 2)),
                    step=1.0,
                    value=float(np.round(np.rad2deg(self.robot.q[j]), 2)),
                    desc=f"{name} Joint {j}",
                    unit="&#176;",
                )
                self.env.add(slider)
                self._sliders.append(slider)

        # Joint step size (deg)
        joint_step = swift.Slider(
            lambda x: self._set_joint_step(x),
            min=0.1,
            max=20.0,
            step=0.1,
            value=float(self.joint_step_deg),
            desc="Joint step (deg)",
            unit="&#176;",
        )
        self.env.add(joint_step)
        self._widgets.append(joint_step)

        # Joint selector using Radio
        try:
            from swift import Radio

            joint_names = [
                f"J{i}"
                for i, l in enumerate(getattr(self.robot, "links", []))
                if getattr(l, "isjoint", False)
            ]
            radio = Radio(
                lambda idx: self._on_joint_select(idx),
                desc="Select joint",
                options=joint_names,
                checked=[True] + [False] * (len(joint_names) - 1) if joint_names else [],
            )
            self.env.add(radio)
            self._widgets.append(radio)
        except Exception:
            try:
                from swift import Checkbox

                joint_names = [
                    f"J{i}"
                    for i, l in enumerate(getattr(self.robot, "links", []))
                    if getattr(l, "isjoint", False)
                ]
                check = Checkbox(
                    lambda checked: self._on_joint_select(self._first_true_index(checked)),
                    desc="Select joint",
                    options=joint_names,
                    checked=[True] + [False] * (len(joint_names) - 1) if joint_names else [],
                )
                self.env.add(check)
                self._widgets.append(check)
            except Exception:
                pass

        # Global jog buttons operate on selected joint
        sel_minus = swift.Button(lambda _=None: self._jog_joint(self._sel_joint, -1), desc="J-")
        sel_plus = swift.Button(lambda _=None: self._jog_joint(self._sel_joint, +1), desc="J+")
        self.env.add(sel_minus)
        self.env.add(sel_plus)
        self._widgets += [sel_minus, sel_plus]

    def _build_cartesian_jogs(self):
        # XYZ step (mm)
        xyz_step = swift.Slider(
            lambda x: self._set_xyz_step_mm(x),
            min=1,
            max=200,
            step=1,
            value=int(self.xyz_step_m * 1000),
            desc="XYZ step (mm)",
            unit="mm",
        )
        self.env.add(xyz_step)
        self._widgets.append(xyz_step)

        # RPY step (deg)
        rpy_step = swift.Slider(
            lambda x: self._set_rpy_step(x),
            min=1,
            max=30,
            step=1,
            value=int(self.rpy_step_deg),
            desc="RPY step (deg)",
            unit="&#176;",
        )
        self.env.add(rpy_step)
        self._widgets.append(rpy_step)

        # Frame toggle
        frame_btn = swift.Button(lambda _=None: self._toggle_frame(), desc=f"Frame: {self.frame}")
        self.env.add(frame_btn)
        self._widgets.append(frame_btn)
        self._frame_btn = frame_btn

        # XYZ jog buttons
        for ax in ("X", "Y", "Z"):
            b1 = swift.Button(lambda _=None, a=ax: self._jog_xyz(a, -1), desc=f"-{ax}")
            b2 = swift.Button(lambda _=None, a=ax: self._jog_xyz(a, +1), desc=f"+{ax}")
            self.env.add(b1)
            self.env.add(b2)
            self._widgets += [b1, b2]

        # Rx/Ry/Rz jog buttons
        for ax in ("Rx", "Ry", "Rz"):
            b1 = swift.Button(lambda _=None, a=ax: self._jog_rpy(a, -1), desc=f"-{ax}")
            b2 = swift.Button(lambda _=None, a=ax: self._jog_rpy(a, +1), desc=f"+{ax}")
            self.env.add(b1)
            self.env.add(b2)
            self._widgets += [b1, b2]

        look_btn = swift.Button(lambda _=None: self._toggle_record(), desc="Look & Listen")
        exam_btn = swift.Button(lambda _=None: self._start_exam(), desc="Exam Time")
        self.env.add(look_btn)
        self.env.add(exam_btn)
        self._widgets += [look_btn, exam_btn]
        self._look_btn = look_btn

    # Actions ------------------------------------------------------------
    def _on_joint_slider(self, j, deg):
        q = np.array(self.robot.q, dtype=float)
        q[j] = np.deg2rad(float(deg))
        self.robot.q = q.tolist()
        if hasattr(self.robot, "_update_3dmodel"):
            self.robot._update_3dmodel()

    def _set_joint_step(self, deg):
        try:
            self.joint_step_deg = float(deg)
        except Exception:
            pass

    def _on_joint_select(self, val):
        # val may be int (Radio) or list[bool] (Checkbox). Normalize to index
        try:
            if isinstance(val, int):
                self._sel_joint = int(val)
            elif isinstance(val, (list, tuple)):
                self._sel_joint = self._first_true_index(val)
        except Exception:
            pass

    def _first_true_index(self, seq):
        for i, v in enumerate(seq):
            if v:
                return i
        return 0

    def _set_xyz_step_mm(self, mm):
        try:
            self.xyz_step_m = float(mm) / 1000.0
        except Exception:
            pass

    def _set_rpy_step(self, deg):
        try:
            self.rpy_step_deg = float(deg)
        except Exception:
            pass

    def _toggle_frame(self):
        self.frame = "base" if self.frame == "tool" else "tool"
        try:
            if self._frame_btn:
                self._frame_btn.desc = f"Frame: {self.frame}"
        except Exception:
            pass

    def _jog_joint(self, j, sign):
        q = np.array(self.robot.q, dtype=float)
        q[j] += np.deg2rad(self.joint_step_deg) * float(sign)
        self.robot.q = q.tolist()
        if hasattr(self.robot, "_update_3dmodel"):
            self.robot._update_3dmodel()

    def _ee_pose(self):
        try:
            T = self.robot.fkine(self.robot.q)
            return T if isinstance(T, SE3) else SE3(T.A)
        except Exception:
            return None

    def _solve_ik(self, T_target):
        try:
            sol = self.robot.ikine_LM(T_target, q0=self.robot.q)
            if getattr(sol, "success", False):
                return np.array(sol.q, dtype=float)
        except Exception:
            return None
        return None

    def _apply_q(self, q):
        self.robot.q = q.tolist()
        if hasattr(self.robot, "_update_3dmodel"):
            self.robot._update_3dmodel()

    def _jog_xyz(self, axis, sign):
        T = self._ee_pose()
        if T is None:
            return
        d = self.xyz_step_m * float(sign)
        Td = SE3.Tx(d) if axis == "X" else SE3.Ty(d) if axis == "Y" else SE3.Tz(d)
        T_target = (T * Td) if self.frame == "tool" else (Td * T)
        q = self._solve_ik(T_target)
        if q is not None:
            self._apply_q(q)

    def _jog_rpy(self, axis, sign):
        T = self._ee_pose()
        if T is None:
            return
        ang = np.deg2rad(self.rpy_step_deg) * float(sign)
        if axis == "Rx":
            Td = SE3.Rx(ang)
        elif axis == "Ry":
            Td = SE3.Ry(ang)
        else:
            Td = SE3.Rz(ang)
        T_target = (T * Td) if self.frame == "tool" else (Td * T)
        q = self._solve_ik(T_target)
        if q is not None:
            self._apply_q(q)

    # Recording / playback -----------------------------------------------
    def _toggle_record(self):
        if self._recording:
            self._recording = False
            if self._look_btn:
                self._look_btn.desc = "Learning..."
        else:
            self._recorded_traj = []
            self._recording = True
            self._playback = False
            self._playback_index = 0
            if self._look_btn:
                self._look_btn.desc = "Learning..."

    def _start_exam(self):
        if self._recording:
            self._recording = False
            if self._look_btn:
                self._look_btn.desc = "Learning..."

        if not self._recorded_traj:
            print("No trajectory recorded yet.")
            return

        self._playback = True
        self._playback_index = 0
        if self._look_btn:
            self._look_btn.desc = "Performing in process!"

    def tick(self, dt=0.02):
        """Call this regularly to record or playback while the env steps."""
        if not self.robot:
            return

        if self._recording:
            self._recorded_traj.append(np.array(self.robot.q, dtype=float))

        if self._playback:
            if not self._recorded_traj:
                self._playback = False
                if self._look_btn:
                    self._look_btn.desc = "Learning..."
                return

            if self._playback_index < len(self._recorded_traj):
                q = self._recorded_traj[self._playback_index]
                self.robot.q = q.tolist()
                if hasattr(self.robot, "_update_3dmodel"):
                    self.robot._update_3dmodel()
                self._playback_index += 1
            else:
                self._playback = False
                if self._look_btn:
                    self._look_btn.desc = "Learning..."


if __name__ == "__main__":
    print("TeachPendant ready.")
