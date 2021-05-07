from scipy.linalg import logm, expm
from scipy.spatial.transform import Rotation as R
import pybullet as p
from utils import *


class Kinematic():
    def __init__(self, arm, origin_pose, origin_positions):
        self.arm = arm
        self.origin_positions = origin_positions
        self.origin_pose = origin_pose

        self.w = []
        self.p = []
        for i in range(len(self.arm.joint_idxes)):
            idx = self.arm.joint_idxes[i]
            _, name, _, _, _, _, _, _, _, _, _, _, child_name, axis, _, _, _ = p.getJointInfo(self.arm.arm, idx)
            _, ori, _, _, pos, _ = p.getLinkState(self.arm.arm, idx)  # joint and child link in same frame
            r_l2w = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
            self.w.append(np.array(axis) @ r_l2w.T)
            self.p.append(np.array(pos))
            print(self.w[-1])


    def fk(self, joints_positions):
        diff = np.array(joints_positions) - self.origin_positions
        mat = np.eye(4)
        for i in range(len(self.arm.joint_idxes)):
            t = np.concatenate([-np.cross(self.w[i], self.p[i]), self.w[i]], axis=0)
            T = expm(twist2mat(t) * diff[i])
            mat = np.matmul(mat, T)
        return np.matmul(mat, self.origin_pose)

    def Ad(self, mat):
        ret = np.eye(6)
        R = mat[:3, :3]
        t = mat[:3, 3]
        ret[:3, :3] = R
        ret[:3, 3:] = np.matmul(screw(t), R)
        ret[3:, 3:] = R
        return ret

    def compute_jacobian(self, joints_positions):
        diff = np.array(joints_positions - self.origin_positions)
        J = np.zeros([6, len(self.arm.joint_idxes)])
        mat = np.eye(4)
        for i in range(len(self.arm.joint_idxes)):
            t = np.concatenate([-np.cross(self.w[i], self.p[i]), self.w[i]], axis=0)
            t_m = twist2mat(t)
            Ad = self.Ad(mat)
            J[:, i] = np.matmul(Ad, t.T)
            T = expm(t_m * diff[i])
            mat = np.matmul(mat, T)
        return J

    def ik_DLS(self, T_target, lambd=1, max_iter=100, threshold=1e-4, init_joints_positions=None):
        # random start joints positions
        degree = len(self.arm.joint_idxes)
        if init_joints_positions is not None:
            current_joints_positions = init_joints_positions
        else:
            current_joints_positions = np.array(self.arm.get_joint_positions()) + np.random.randn(degree) * np.pi * (5/180)

        # iteration
        # T_target = e^[V] @ T_cur
        T_cur = self.fk(current_joints_positions)
        e_V = np.matmul(T_target, np.linalg.inv(T_cur))
        n = np.linalg.norm(e_V - np.eye(4))
        cnt = 0
        while n >= threshold and cnt < max_iter:
            # v = J @ theta_dot
            v = mat2twist(logm(e_V).real)
            J = self.compute_jacobian(current_joints_positions)

            # damped least squares
            J = np.concatenate([J, np.eye(degree) * lambd], axis=0)
            v = np.concatenate([v, np.zeros(degree)], axis=0)
            theta_dot = np.matmul(np.linalg.inv(np.matmul(J.T, J)), np.matmul(J.T, v))

            # update
            current_joints_positions += theta_dot.reshape(-1)

            T_cur = self.fk(current_joints_positions)
            e_V = np.matmul(T_target, np.linalg.inv(T_cur))
            n = np.linalg.norm(e_V - np.eye(4))
            cnt += 1
        print("ik iteration", cnt)
        return self.clip_joint(current_joints_positions)


    def ik_pinv(self, T_target, max_iter=100, threshold=1e-4, init_joints_positions=None):
        # random start joints positions
        degree = len(self.arm.joint_idxes)
        if init_joints_positions is not None:
            current_joints_positions = init_joints_positions
        else:
            current_joints_positions = np.array(self.arm.get_joint_positions()) + np.random.randn(degree) * np.pi * (5/180)

        # iteration
        # T_target = e^[V] @ T_cur
        T_cur = self.fk(current_joints_positions)
        e_V = np.matmul(T_target, np.linalg.inv(T_cur))
        n = np.linalg.norm(e_V - np.eye(4))
        cnt = 0
        while n >= threshold and cnt < max_iter:
            # v = J @ theta_dot
            v = mat2twist(logm(e_V).real)
            J = self.compute_jacobian(current_joints_positions)

            # pseudo invers bu SVD
            U, d, V_T = np.linalg.svd(J, full_matrices=True)
            D_inv = np.eye(d.shape[0]) * (1/d)
            J_inv = np.matmul(np.matmul(V_T.T, D_inv), U.T)
            theta_dot = np.matmul(J_inv, v)

            # update
            current_joints_positions += theta_dot

            T_cur = self.fk(current_joints_positions)
            e_V = np.matmul(T_target, np.linalg.inv(T_cur))
            n = np.linalg.norm(e_V - np.eye(4))
            cnt+=1
        print("ik iteration", cnt)
        return self.clip_joint(current_joints_positions)

    def clip_joint(self, joint_positions):
        joint_positions = joint_positions % (np.pi * 2)
        filter = np.abs(joint_positions) > np.pi
        joint_positions[filter] = joint_positions[filter] - np.pi * 2
        return joint_positions