import sys
import os

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
import cv2

sys.path.append(os.path.dirname(__file__))
from camera import Camera


class Realsense:
    def __init__(self):
        self.id = p.createMultiBody()
        self.color_cam = Camera(width=640, height=480, fov=55.5, near_val=0.01, far_val=10)
        self.left_cam = Camera(width=640, height=480, fov=79.5, near_val=0.01, far_val=10)
        self.right_cam = Camera(width=640, height=480, fov=79.5, near_val=0.01, far_val=10)
        self.proj_cam = Camera(width=640, height=480, fov=90, near_val=0.01, far_val=10)
        self.T_c2b, self.T_l2b, self.T_r2b, self.T_p2b = np.eye(4), np.eye(4), np.eye(4), np.eye(4)
        self.T_c2b[0, 3] = -0.0325
        self.T_l2b[0, 3] = -0.0175
        self.T_r2b[0, 3] = 0.0325
        self.T_p2b[0, 3] = 0.0115

        self.baseline = 0.05 # m
        self.unit = 1 #
        self.f = self.left_cam.intrinsic[0,0]
        disparities = 128
        block_size = 31
        self.matcher = cv2.StereoBM_create(numDisparities=disparities, blockSize=block_size)

        I, J = np.meshgrid(np.arange(self.proj_cam.height), np.arange(self.proj_cam.width), indexing='ij')
        proj_points = np.stack([J, I, np.ones_like(I)], axis=-1)
        self.proj_points = proj_points @ np.linalg.inv(self.proj_cam.intrinsic).T
        self.num_sample = 100000
        self.sample_i = np.random.randint(0, self.proj_cam.height, self.num_sample)
        self.sample_j = np.random.randint(0, self.proj_cam.width, self.num_sample)
        self.ksize = 11
        self.sigma = 1

    def set_pose(self, T_b2w):
        p.resetBasePositionAndOrientation(self.id, T_b2w[:3, 3], R.from_matrix(T_b2w[:3, :3]).as_quat())

    def project(self, img, depth, intr, T_proj2cur, proj_points, white_noise_idx, black_noise_idx):
        points = proj_points.copy()
        points[..., 0] += T_proj2cur
        D = points[..., 2]
        points = points / points[..., [2]] @ intr.T
        I = points[..., 1]
        J = points[..., 0]
        valid = ((I >= 0) * (I < depth.shape[0])
                * (J >= 0) * (J < depth.shape[1]))
        valid_D = D[valid]
        valid_I = I[valid].astype(int)
        valid_J = J[valid].astype(int)
        relat_D = depth[valid_I, valid_J]
        mask = (valid_D - relat_D)**2 < 10
        i = valid_I[mask]
        j = valid_J[mask]

        splot = np.zeros_like(depth)
        splot[i, j] = 255
        splot = cv2.GaussianBlur(splot, (self.ksize, self.ksize), self.sigma)

        splot = splot / splot.max() * 255
        ret = np.clip(img + splot, 0, 255).astype(np.uint8)

        ret[white_noise_idx] = 255
        ret[black_noise_idx] = 0
        return ret

    def get_image(self, noise=False):
        pos, ori = p.getBasePositionAndOrientation(self.id)
        T_b2w = np.eye(4)
        T_b2w[:3, :3] = R.from_quat(ori).as_matrix()
        T_b2w[:3, 3] = pos
        self.color_cam.set_pose(T_b2w @ self.T_c2b)
        self.left_cam.set_pose(T_b2w @ self.T_l2b)
        self.right_cam.set_pose(T_b2w @ self.T_r2b)
        self.proj_cam.set_pose(T_b2w @ self.T_p2b)

        color, color_depth, seg = self.color_cam.get_image()

        if not noise:
            return color, color_depth, seg

        proj_depth = self.proj_cam.get_image()[1]
        left, left_depth, _ = self.left_cam.get_image()
        right, right_depth, _ = self.right_cam.get_image()
        left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        # get projected points
        proj_points = self.proj_points * proj_depth[..., None]
        proj_points = proj_points[self.sample_i, self.sample_j]
        noise = np.random.randn(*left.shape)
        white_noise_idx = noise>3
        black_noise_idx = noise<-3

        # project to left camera
        left = self.project(left, left_depth, self.left_cam.intrinsic, 0.029, proj_points, white_noise_idx, black_noise_idx)
        cv2.imshow("left", left)
        cv2.waitKey(1)

        # project to right camera
        right = self.project(right, right_depth, self.right_cam.intrinsic, -0.021, proj_points, white_noise_idx, black_noise_idx)

        # stereo
        disparity = self.matcher.compute(left, right) / 16
        depth = np.zeros_like(left).astype(float)
        depth[disparity > 0] = (self.f * self.baseline) / (self.unit * disparity[disparity > 0])

        # align to color
        depth_aligned = np.zeros_like(depth)
        I, J = np.meshgrid([np.arange(depth.shape[0])], np.arange(depth.shape[1]), indexing='ij')
        points = np.stack([J, I, np.ones_like(I)], axis=-1)
        points = points @ np.linalg.inv(self.color_cam.intrinsic).T
        points[..., 0] = (-0.015 + points[..., 0]*color_depth) / color_depth
        idx = points @ self.left_cam.intrinsic.T
        i = idx[..., 1]
        j = idx[..., 0]
        valid = ((i >= 0) * (i < self.color_cam.height)
                * (j >= 0) * (j < self.color_cam.width))
        depth_aligned[valid] = depth[i[valid].astype(int), j[valid].astype(int)]

        return color, depth_aligned, seg
