import pybullet as p
import pybullet_data
import os
from scipy.spatial.transform import Rotation as R

import numpy as np
class Camera:
    def __init__(self):
        self.pose = np.eye(4)
        self.focus_distance = 1
        self.set_pose(self.pose)
        self.width = 480
        self.height = 480
        self.projection_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                         # aspect=self.height/self.width,
                                                         aspect=self.width/self.height,
                                                         nearVal=0.1,
                                                         farVal=10)

    def set_pose(self, pose):
        self.eye_position = pose[:3, 3]
        self.up_vector = -pose[:3, 1]
        self.target_position = self.eye_position + pose[:3, 2]*self.focus_distance
        # self.eye_position = np.array([0,0,-1])
        # self.up_vector = np.array([0,-1,0])
        # self.target_position = np.array([0,0,0])
        self.view_matrix = p.computeViewMatrix(cameraEyePosition=self.eye_position,
                                               cameraTargetPosition=self.target_position,
                                               cameraUpVector=self.up_vector)

    def get_image(self):
        return p.getCameraImage(width=self.width, height=self.height,
                                viewMatrix=self.view_matrix,
                                projectionMatrix=self.projection_matrix,
                                lightDirection=[0,0,2])

if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=False)

    plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0,0,-1])
    ur5 = p.loadURDF("forearm.urdf")


    camera = Camera()
    pose = np.eye(4)
    pose[:3, :3] = R.from_euler('xyz', [180, 0, 0], degrees=True).as_matrix()
    pose[:3, 3] = np.array([0,0,1])
    camera.set_pose(pose)

    while True:
        p.stepSimulation()
        data = camera.get_image()