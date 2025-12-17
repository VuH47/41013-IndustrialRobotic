"""
1. TeachPendant w HM47 rbt for manual jogging.
2. TeachPendant working together with PnPTool for a pick/place cord.

Run either block by executing this file directly and choosing an option.
"""

from __future__ import annotations
import sys
import time
from pathlib import Path
import swift

# Allow running the script directly without installing the package.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Tools.PnP_Tool import PnPTool  
from Tools.TeachPendant import TeachPendant  
from IRB6740SW1 import HM47  


def run_teachpendant_demo():
    """
    TeachPendant test.
    Launch Swift, add HM47, attach the teach pendant, and keep stepping.
    """
    env = swift.Swift()
    env.launch(realtime=True)

    robot = HM47()
    robot.add_to_env(env)

    pendant = TeachPendant().add_to_env(env, robot)

    try:
        while _env_is_open(env):
            pendant.tick()
            env.step(0.02)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        _close_env(env)


def run_pnp_demo():
    """
    Combined TeachPendant + PnPTool test.
    """
    env = swift.Swift()
    env.launch(realtime=True)

    robot = HM47()
    robot.add_to_env(env)

    pendant = TeachPendant().add_to_env(env, robot)
    tool = PnPTool(robot, env, pendant)

    try:
        tool.run_interactive()
    finally:
        _close_env(env)


def _env_is_open(env) -> bool:
    status = getattr(env, "isopen", None)
    if callable(status):
        try:
            return bool(status())
        except TypeError:
            return True
    if isinstance(status, bool):
        return status
    return True


def _close_env(env):
    close_fn = getattr(env, "close", None)
    if callable(close_fn):
        close_fn()
        return
    hold_fn = getattr(env, "hold", None)
    if callable(hold_fn):
        hold_fn()
    else:
        try:
            env.quit()
        except AttributeError:
            pass


def main():
    print("Select demo:")
    print("  1) TeachPendant only")
    print("  2) TeachPendant + PnPTool interactive CLI")
    choice = input("Choice [1/2]: ").strip() or "1"

    if choice == "2":
        run_pnp_demo()
    else:
        run_teachpendant_demo()


if __name__ == "__main__":
    main()
