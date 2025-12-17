# Minh HUYNH aka 47HMperformance (47hm.com)
# Course: Industrial Robotics - UTS

import time
import threading
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion as UQ


class PnPTool:
    """Pick-and-place helper for Swift + Robotics Toolbox."""

    def __init__(self, robot, env, pendant, target_object=None):
        self.robot = robot
        self.env = env
        self.pendant = pendant
        self.target_object = target_object

        # Pick/drop poses
        self.pick_pose = None
        self.drop_pose = None

        # Internal state
        self._run_loop = False
        self._ui_thread = None
        self._attached = False
        self._T_TO = np.eye(4)

        print(f"\nPnPTool initialized for {robot.name}")


    # --------------------------------------------------
    # Printer Functions 
    def step_env(self, dt: float = 0.02):
        if hasattr(self.robot, "_update_3dmodel"):
            self.robot._update_3dmodel()
        self.env.step(dt)

    def print_current_pose(self):
        T = self.robot.fkine(self.robot.q)
        pos = T.t
        quat_wxyz = UQ(T.R).vec  # (w, x, y, z)
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        rpy_rad = T.rpy()
        rpy_deg = np.rad2deg(rpy_rad)

        print("\n--- Current Tool Pose ---")
        print(f"XYZ (m):       {np.round(pos, 6)}")
        print(f"Quat (x,y,z,w): {np.round(quat_xyzw, 6)}")
        print(f"RPY  (rad):    {np.round(rpy_rad, 6)}")
        print(f"RPY  (deg):    {np.round(rpy_deg, 3)}")
        print("-------------------------\n")


    # --------------------------------------------------
    #Loop to maintain Teachpendant working
 

    def _ui_loop(self):
        while self._run_loop:
            if self.pendant and hasattr(self.pendant, 'tick'):
                self.pendant.tick(0.05)
            self.step_env(0.05)
            time.sleep(0.05)

    def start_ui_loop(self):
        self._run_loop = True
        self._ui_thread = threading.Thread(target=self._ui_loop, daemon=True)
        self._ui_thread.start()

    def stop_ui_loop(self):
        self._run_loop = False
        if self._ui_thread:
            self._ui_thread.join(timeout=1.0)


    # --------------------------------------------------
    # Terminal command


    def run_interactive(self):
        print("\n" + "="*60)
        print("Interactive Pick and Place Mode")
        print("="*60)
        print("Commands:")
        print("  [Enter] or 'p' - Print current pose")
        print("  'run'          - Execute pick/place operation")
        print("  'q'            - Quit")
        print("="*60 + "\n")

        # Start background loop
        self.start_ui_loop()

        try:
            while True:
                cmd = input("PnP > ").strip().lower()

                if cmd in ("", "p"):
                    self.print_current_pose()

                elif cmd == "run":
                    print("\nExiting interactive mode to run pick/place")
                    break

                elif cmd == "q":
                    self.stop_ui_loop()
                    self.env.hold()
                    print("\nExiting...\n")
                    raise SystemExit

                else:
                    print("Unknown command. Use Enter / run / q.")

        finally:
            self.stop_ui_loop()


    # --------------------------------------------------
    # Pick and Place Execution with IK


    def execute_pick_place(self, pick_pos, pick_quat_xyzw, drop_pos, drop_quat_xyzw):
        # Convert to SE3
        pick_uq = UQ([pick_quat_xyzw[3], *pick_quat_xyzw[:3]])
        drop_uq = UQ([drop_quat_xyzw[3], *drop_quat_xyzw[:3]])
        T_pick = SE3.Rt(pick_uq.R, pick_pos)
        T_drop = SE3.Rt(drop_uq.R, drop_pos)

        q_start = self.robot.q.copy()
        app_offset = SE3(0, 0, 0.05)

        # Solve IK
        print("\n" + "="*60)
        print("SOLVING INVERSE KINEMATICS")
        print("="*60)

        sol_app_pick = self.robot.ikine_LM(T_pick * app_offset, q0=q_start)
        if not sol_app_pick.success:
            raise RuntimeError("IK failed for pick approach")
        print("Pick approach IK solved")

        sol_pick = self.robot.ikine_LM(T_pick, q0=sol_app_pick.q)
        if not sol_pick.success:
            raise RuntimeError("IK failed for pick")
        print("Pick IK solved")

        sol_app_drop = self.robot.ikine_LM(T_drop * app_offset, q0=sol_pick.q)
        if not sol_app_drop.success:
            raise RuntimeError("IK failed for drop approach")
        print("Drop approach IK solved")

        sol_drop = self.robot.ikine_LM(T_drop, q0=sol_app_drop.q)
        if not sol_drop.success:
            raise RuntimeError("IK failed for drop")
        print("Drop IK solved")

        print("="*60)
        print("EXECUTING MOTION")
        print("="*60 + "\n")

        # Execute
        self._move(q_start, sol_app_pick.q, 50, "Move to pick approach")
        self._move(sol_app_pick.q, sol_pick.q, 30, "Descend to pick")

        # Attach
        if self.target_object:
            self._T_TO = np.linalg.inv(self.robot.fkine(self.robot.q).A) @ self.target_object.T
            self._attached = True
            print("Object attached")

        self._move(sol_pick.q, sol_app_drop.q, 60, "Move to drop", True)
        self._move(sol_app_drop.q, sol_drop.q, 30, "Descend to drop", True)

        # Detach
        if self.target_object and self._attached:
            self.target_object.T = self.robot.fkine(self.robot.q).A @ self._T_TO
            self._attached = False
            print("Object released")

        self._move(sol_drop.q, q_start, 60, "Return to start")

        print("\n" + "="*60)
        print("PICK AND PLACE COMPLETED!")
        print("="*60 + "\n")


    def _move(self, q_from, q_to, steps, desc="", move_obj=False):
        if desc:
            print(f"{desc}...", end=' ', flush=True)

        qtraj = rtb.jtraj(q_from, q_to, steps).q

        for q in qtraj:
            self.robot.q = q

            if move_obj and self._attached and self.target_object:
                self.target_object.T = self.robot.fkine(self.robot.q).A @ self._T_TO

            self.step_env()

        if desc:
            print("Done")


# ============================================================================
# Example Usage
# ============================================================================

if __name__ == "__main__":
 print('hello word')
