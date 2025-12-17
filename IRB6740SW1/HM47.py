##  @file
#   @details ABB6740-270-280 SW1 defined by matching DH paramter with ABB dimension with 3D model
#   @this is self create and for the purpose of Education 
#   @ Author: Minh HUYNH 


import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
import spatialgeometry
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
import time
import os
import numpy as np
from math import pi
import matplotlib.pyplot as plt
from spatialgeometry import Cylinder


# -----------------------------------------------------------------------------------#
class HM47(DHRobot3D):   
    def __init__(self):    
        # DH links
        links = self._create_DH()     

        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'Base', 
                            link1 = 'Link1e', 
                            link2 = 'Link2e', 
                            link3 = 'Link3e', 
                            link4 = 'Link4e', 
                            link5 = 'Link5e', 
                            link6 = 'Link6e')
        # link3D_names = dict(link0 = 'Base',
        #                     link1 = 'LinkD1e', 
        #                     link2 = 'Link1e', 
        #                     link3 = 'Link2e', 
        #                     link4 = 'Link3e', 
        #                     link5 = 'Link4e', 
        #                     link6 = 'Link5e', 
        #                     link7 = 'Link6e')
        #,pump mount with link1 rotation link2 = 'LinkD1e

        qtest =  [ 0, 0, 0, 0, 0, 0]
        qtest_transforms = [spb.transl(0,0.04,0) ,
                            #spb.transl(-0.3525, 0.05 , 0.67751) , #linkd1@ spb.tr2rpy(np.deg2rad(-80.264),np.deg2rad(30),np.deg2rad(125.26), order ='xyz')
                            spb.transl(0.14,0.084,0.6) ,             #l1
                            spb.transl(0.390 ,-0.081, 0.745),        #l2
                            spb.transl(0.381,0.068 ,1.965),          #l3
                            spb.transl(1.5825, 0.068  ,2.1525),      #l4
                            spb.transl(1.5825,0.068 , 2.205),        #l5 
                            spb.transl(1.8620, 0.048  ,2.229)]
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'HM47', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms ) 
        self.q = qtest        

    def _create_DH(self):
        # links = [rtb.RevoluteDH( a= 0, alpha= pi/2, qlim= [-2.967, 2.967])]
        a = [0.3212, 1.22949, 0.2, 0, 0, -0.223]
        d = [0.7821, 0, 0, 1.380, 0, 0.2011]
        alpha = [-pi/2, 0, -pi/2, pi/2,-pi/2, 0]
        offset = [0, -pi/2,0,0,0,-pi]
        qlim = [(-2.967, 2.967), (-1.57, 1.22), (-2.967, 1.134), (-5.236, 5.236), (-2.094, 2.094), (-6.283, 6.283)]
        links = []
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlim[i],offset=offset[i])  
            links.append(link)
        return links
    
  
    def test(self):
        env = swift.Swift()
        env.launch(realtime= True)      
        self.q = self._qtest  
        self.base = SE3(0,0,0)
        self.add_to_env(env)


#--------------------------------test tung cai---------------------------------------------------------------
        # dq = np.array([0.9, 0.8, 0.7 , 0.6, 0, 0])
        # q_goal = (np.array(self.q)+ dq).tolist()
        # qtraj = rtb.jtraj(self.q, q_goal, 100).q
        # for q in qtraj:
        #     self.q = q
        #     env.step(0.03)
        # env.hold()
        # time.sleep(3)


        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        time.sleep(3)
        env.hold()
    
# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":  
    r = HM47()
    r.test()

