import pybullet as p

import numpy as np
class Camera:
    def __init__(self):
        self.pose = np.eye(4)
        self.focus_distance = 1
        self.set_pose(self.pose)
        self.width = 1280
        self.height = 960
        self.projection_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                         aspect=self.width/self.height,
                                                         nearVal=0.01,
                                                         farVal=10)


    def set_pose(self, pose):
        self.eye_position = pose[:3, 3]
        self.up_vector = -pose[:3, 1]
        self.target_position = self.eye_position + pose[:3, 2]*self.focus_distance
        self.view_matrix = p.computeViewMatrix(cameraEyePosition=self.eye_position,
                                               cameraTargetPosition=self.target_position,
                                               cameraUpVector=self.up_vector)

    def get_image(self):
        return p.getCameraImage(
            width=self.width, height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            shadow=True,
            lightDirection=[1,1,1],
            lightDistance=-1,
            lightAmbientCoeff = -1,
            lightSpecularCoeff = -1,
            lightDiffuseCoeff = -1,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
            # renderer=p.ER_TINY_RENDERER
        )
