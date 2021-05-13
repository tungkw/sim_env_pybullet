import pybullet as p
import pybullet_data
from pybullet_utils import urdfEditor as ed
import os

class Suction:
    def __init__(self, parent_obj=None, parent_link=-1):


if __name__ == '__main__':
    from ur5 import UR5
    import numpy as np
    import time

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.setGravity(0, 0, -10)

    p.loadURDF("plane.urdf")  # , [0,0,-2])

    arm = UR5()
    suction = Suction(arm.arm, arm.tool0)

    for i in range(100):
        p.stepSimulation()

    # joint_positions = np.array([0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0])
    # arm.set_joint_target_positions(joint_positions)
    while True:
        p.stepSimulation()
        time.sleep(0.05)