
from pybullet_utils import urdfEditor as ed
from scipy.linalg import logm, expm
from scipy.spatial.transform import Rotation as R

from ur5 import UR5
import pybullet as p
import numpy as np
import os

class UR5Suction(UR5):
    def __init__(self):
        super().__init__(urdf_file=os.path.join(os.path.dirname(__file__), "meshes/ur5/ur5_suction.urdf"))
        p.setAdditionalSearchPath(os.path.dirname(__file__))
        pose = self.get_pose()
        ori = pose[3:]
        pos = pose[:3] + R.from_quat(ori).as_matrix()[:3, 2] * 0.05
        self.cup = p.loadSoftBody(
            fileName=os.path.join(os.path.dirname(__file__), "meshes/suction/cup.obj"),
            simFileName=os.path.join(os.path.dirname(__file__), "meshes/suction/cup.vtk"),
            basePosition=pos,
            baseOrientation=ori,
            scale=1,
            mass=0.1,

            useBendingSprings=1,
            useMassSpring=1,
            springElasticStiffness=1e2,
            springDampingStiffness=1e-1,
            springDampingAllDirections=1,

            useSelfCollision=1,
            frictionCoeff=.5,
            useFaceContact=1,
            collisionMargin=0.0001,
            repulsionStiffness=800,

            useNeoHookean=1,
            NeoHookeanMu=1e4,
            NeoHookeanLambda=1e2,
            NeoHookeanDamping=1e-2,
        )
        p.changeVisualShape(self.cup, -1, rgbaColor=[.5, .5, .5, 1])

        self.num_sample, self.vertices = p.getMeshData(self.cup, flags=p.MESH_DATA_SIMULATION_MESH)
        self.num_sample //= 4
        for i in range(2 * self.num_sample):
            p.createSoftBodyAnchor(self.cup, i, self.arm, self.tool0, self.vertices[i])
        self.grasp_anchor = []

    def check(self):
        _, vertices = p.getMeshData(self.cup, flags=p.MESH_DATA_SIMULATION_MESH)
        cur = vertices[-2*self.num_sample:-self.num_sample]
        # cur = vertices[-self.num_sample:]
        foward = np.array(cur)
        pose = self.get_pose()
        direction = R.from_quat(pose[3:]).as_matrix()[:3, 2]
        foward += direction*0.001
        result = p.rayTestBatch(cur, foward)
        obj = result[0][0]
        for i in range(self.num_sample):
            if result[i][0] == -1:
                # print(1)
                return -1
            if result[i][0] != obj:
                # print(2, result[i][0])
                return -1
            if result[i][2] > 0.01:
                # print(3, result[i][2])
                return -1
        return obj

    def grasp(self):
        obj = self.check()
        if obj != -1:
            _, vertices = p.getMeshData(self.cup, flags=p.MESH_DATA_SIMULATION_MESH)
            cur = vertices[-2*self.num_sample:-self.num_sample]
            for i in range(self.num_sample):
                self.grasp_anchor.append(p.createSoftBodyAnchor(self.cup, i+2*self.num_sample, obj, -1, cur[i]))

    def release(self):
        for anchor in self.grasp_anchor:
            p.removeConstraint(anchor)


if __name__ == '__main__':
    from scipy.spatial.transform import Rotation as R
    import pybullet_data
    import time

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.setGravity(0, 0, -10)

    motorsIds = []
    motorsIds.append(p.addUserDebugParameter("posX", 0.2, 0.7, 0.5))
    motorsIds.append(p.addUserDebugParameter("posY", -0.2, 0.2, 0.0))
    motorsIds.append(p.addUserDebugParameter("yaw", 0, 0.7, 0.5))
    motorsIds.append(p.addUserDebugParameter("grasp", 0, 1, 0))

    p.loadURDF("plane.urdf")  # , [0,0,-2])
    box = p.loadURDF("cube.urdf", [0.5, -0.055, 0], globalScaling=0.1)
    box2 = p.loadURDF("cube.urdf", [0.5, 0.055, 0], globalScaling=0.1)

    arm = UR5Suction()

    # joint_positions = np.array([0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0])
    # arm.set_joint_target_positions(joint_positions, True)

    while True:
        action = []
        for motorId in motorsIds[:-1]:
            action.append(p.readUserDebugParameter(motorId))
        pose = arm.get_pose()
        pose[:3] = action
        arm.set_pose(pose)

        p.stepSimulation()

        grasp = p.readUserDebugParameter(motorsIds[-1])
        if grasp == 1:
            arm.grasp()
        else:
            arm.release()