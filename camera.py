import pybullet as p

import numpy as np


class Camera:
    def __init__(self, width=640, height=480, fov=60, near_val=0.01, far_val=10):
        self.pose = np.eye(4)
        self.focus_distance = 1
        self.set_pose(np.eye(4))
        self.width = width
        self.height = height
        self.near_val = near_val
        self.far_val = far_val
        self.projection_matrix = p.computeProjectionMatrixFOV(fov=fov,  # width angle
                                                              aspect=self.width / self.height,
                                                              nearVal=near_val,
                                                              farVal=far_val)
        f = height / 2 / np.tan(fov / 2 / 180 * np.pi)
        self.intrinsic = np.array([
            [f, 0, width / 2],
            [0, f, height / 2],
            [0, 0, 1]
        ])

    def set_pose(self, pose):
        self.pose = pose
        self.eye_position = pose[:3, 3]
        self.up_vector = -pose[:3, 1]
        self.target_position = self.eye_position + pose[:3, 2] * self.focus_distance
        p.stepSimulation()
        self.view_matrix = p.computeViewMatrix(cameraEyePosition=self.eye_position,
                                               cameraTargetPosition=self.target_position,
                                               cameraUpVector=self.up_vector)

    def get_image(self):
        p.stepSimulation()
        _, _, color, depth, seg = p.getCameraImage(
            width=self.width, height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            # shadow=False,
            # lightDirection=[1,1,1],
            # lightDistance=-1,
            # lightAmbientCoeff = -1,
            # lightSpecularCoeff = -1,
            # lightDiffuseCoeff = -1,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
            # renderer=p.ER_TINY_RENDERER
        )
        return color, self.far_val * self.near_val / (self.far_val - (self.far_val - self.near_val) * depth), seg


if __name__ == '__main__':
    p.connect(p.GUI)

    cam = Camera()
    print(np.array([np.tan(30/180*np.pi), np.tan(30/180*np.pi)/640*480, 1]) @ cam.intrinsic.T)
