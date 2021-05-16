import pybullet as p
import numpy as np
from scipy.linalg import logm, expm
from scipy.spatial.transform import Rotation as R
import os

class UR5:
    def __init__(self, urdf_file=os.path.join(os.path.dirname(__file__), "meshes", "ur5", "ur5.urdf")):
        self.arm = p.loadURDF(urdf_file, flags=p.URDF_USE_SELF_COLLISION)
        self.joint_names = ["shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint"]
        self.joint_idxes = []
        self.joint_w = []
        self.joint_p = []
        self.joint_parent_idx = []
        self.tool0 = 0
        for i in range(p.getNumJoints(self.arm)):
            idx, name, jtype, q_idx, v_idx, _, damping, friction, ll, ul, fmax, vmax, child_name, axis, pos, ori, parent_idx = p.getJointInfo(self.arm, i)
            if str(name, encoding='utf-8') in self.joint_names:
                self.joint_idxes.append(idx)
                p.changeDynamics(self.arm, idx, maxJointVelocity=1, jointDamping=0.1)
            if str(child_name, encoding='utf-8') == 'tool0':
                self.tool0 = idx

        self.init_joint_positions = np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0])
        # self.init_joint_positions = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])
        self.set_joint_target_positions(self.init_joint_positions, wait=True)

    def get_joint_states(self):
        positions = []
        velocities = []
        for state in p.getJointStates(self.arm, self.joint_idxes):
            positions.append(state[0])
            velocities.append(state[1])
        return np.array(positions), np.array(velocities)

    def set_joint_target_positions(self, positions, wait=False):
        # p.setJointMotorControlArray(
        #     bodyUniqueId=self.arm,
        #     jointIndices=self.joint_idxes,
        #     controlMode=p.POSITION_CONTROL,
        #     targetPositions=positions,
        #     # targetVelocities=[0,0,0,0,0,0],
        #     # maxVelocities=[1,1,1,1,1,1],
        # )
        for i, idx in enumerate(self.joint_idxes):
            p.setJointMotorControl2(
                self.arm,
                idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=positions[i],
                targetVelocity=0,
                maxVelocity=1,
            )
        if wait:
            p.stepSimulation()
            while np.any(np.abs(self.get_joint_states()[1]) > 0.01):
                p.stepSimulation()

    def set_pose(self, pose, wait=False):
        joint_target_positions = p.calculateInverseKinematics(
            self.arm,
            self.tool0,
            pose[:3],
            pose[3:],
            maxNumIterations=100
        )
        self.set_joint_target_positions(joint_target_positions, wait=wait)

    def get_pose(self):
        _, _, _, _, pos, ori = p.getLinkState(self.arm, self.tool0)#, computeForwardKinematics=True)
        return np.array(pos + ori)

if __name__ == '__main__':
    from scipy.spatial.transform import Rotation as R
    import time
    # p.connect(p.GUI)
    p.connect(p.DIRECT)

    arm = UR5()
    print()


    # # init_position = np.array([0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0])
    # init_position = np.array([0,-np.pi/10,np.pi/10,0,0,0])
    # # init_position = np.array([0,0,0,0,0,0])
    # arm.set_joint_target_positions(init_position, wait=True)
    test_pose = arm.get_pose()
    print(test_pose)
    # print(arm.kinematic.fk(arm.init_joint_positions))

    # test_pose[:3] = [0., 0.5, 0.5]
    test_pose[:3] = [0.5, 0., 0.5]
    test_pose[3:] = R.from_euler("xyz", [180,0,-90], degrees=True).as_quat()
    print(test_pose)
    # arm.set_joint_target_positions(np.array([0,0,0,0,0,0]), wait=True)
    ts = time.time()
    arm.set_pose(test_pose, wait=True)
    print(time.time() - ts)
    print(arm.get_pose())

    from camera import Camera
    import cv2
    camera = Camera()
    pose = np.eye(4)
    pose[:3, :3] = R.from_euler('xyz', [210, 0, 0], degrees=True).as_matrix()
    pose[:3, 3] = np.array([0.1,-0.5,1.5])
    camera.set_pose(pose)
    cv2.imshow("image", camera.get_image()[2])
    cv2.waitKey()