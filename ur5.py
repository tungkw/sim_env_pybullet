import pybullet as p
import numpy as np

class UR5:
    def __init__(self):
        self.arm = p.loadURDF("./meshes/ur5/ur5.urdf")
        self.joint_names = ["shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint"]
        self.joint_idxes = []
        self.tool0 = 0
        for i in range(p.getNumJoints(self.arm)):
            idx, name, jtype, q_idx, v_idx, _, damping, friction, ll, ul, fmax, vmax, child_name, axis, pos, ori, parent_idx = p.getJointInfo(self.arm, i)
            if str(name, encoding='utf-8') in self.joint_names:
                self.joint_idxes.append(idx)
            if str(child_name, encoding='utf-8') == 'tool0':
                self.tool0 = idx

    def get_joint_positions(self):
        positions = []
        for state in p.getJointStates(self.arm, self.joint_idxes):
            positions.append(state[0])
        return np.array(positions)

    def set_joint_target_positions(self, positions):
        p.setJointMotorControlArray(
            bodyUniqueId=self.arm,
            jointIndices=self.joint_idxes,
            controlMode=p.POSITION_CONTROL,
            targetPositions=positions
        )

    def set_pose(self, pose):
        joint_target_positions = p.calculateInverseKinematics(
            self.arm,
            self.tool0,
            pose[:3],
            pose[3:]
        )
        self.set_joint_target_positions(joint_target_positions)

    def get_pose(self):
        _, _, _, _, pos, ori = p.getLinkState(self.arm, self.tool0)#, computeForwardKinematics=True)
        return pos + ori

if __name__ == '__main__':
    from scipy.spatial.transform import Rotation as R
    p.connect(p.GUI)
    arm = UR5()
    pos = [0.3, 0.3, 0.3]
    ori = R.from_euler('xyz', [180, 0, -90], degrees=True).as_quat()
    arm.set_pose(pos + ori.tolist())
    while True:
        p.stepSimulation()
        print(arm.get_pose())