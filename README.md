# 41013 IndustrialRobotic
<img width="1065" height="611" alt="image" src="https://github.com/user-attachments/assets/eb298df8-66d5-456a-8932-4a77b25720ec" />


My repository provides resources for the Robotics simulation in Python (Swift env), including:

- Folder  "IRB6740 and IRB1660" contain model's link file(dae.) as well as DH parameter.
Tutorials: visit my YTB for tutorial on making your own & implementing your own model in Swift using Blender/MeshLab. 

- File "PnP_Tool.py": A ready to use "Class" for accurate and time-saving Pick & Place tasks; this method contains:

  + Kinematics (FK/IK) for motion planning
  + Transformation geometry (SE3, quaternions(Unit Quaternions,RPY Euler Angles)) for pose representation
  + Trajectory generation for smooth motion.
  + Grasp mechanics via frame transformations.


-  File "TeachPendant.py": A ready to use "Class" to control a single RBT or a 2nd RBT in the same workspace, can do SyncPose aswell. Theory including:
   + Joint space control: all the q(links)
   + Cartesian Space control
   + Tool Frame/Base Frame:using end-effector(ee) to ctrl movements relative to the tool's local coordinates.Using world frame to ctrl movements relative to the fixed robot base
   + "Study & Exam": record and playback individual or multirobot through teach call. 






