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
        for i in range(p.getNumJoints(self.arm)):
            idx, name, jtype, q_idx, v_idx, _, damping, friction, ll, ul, fmax, vmax, child_name, axis, pos, ori, parent_idx = p.getJointInfo(self.arm, i)
            if str(name, encoding='utf-8') in self.joint_names:
                self.joint_idxes.append(idx)

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