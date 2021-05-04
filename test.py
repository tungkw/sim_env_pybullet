import cv2
import pybullet as p
import pybullet_data
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
from camera import Camera
from ur5 import UR5

if __name__ == '__main__':
    client = p.connect(p.GUI)
    # client = p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(enableFileCaching=False)
    print("numpy enabled", p.isNumpyEnabled())

    plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0,0,-0.2])
    plugin = p.loadPlugin(r"C:\Program Files\Python36\Lib\site-packages\examples/SharedMemory/plugins/eglPlugin", "eglRendererPlugin")
    print(plugin)

    camera = Camera()
    arm = UR5()

    print(arm.get_joint_positions())
    arm.set_joint_target_positions([0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0])

    pose = np.eye(4)
    pose[:3, :3] = R.from_euler('xyz', [180, 30, 0], degrees=True).as_matrix()
    pose[:3, 3] = np.array([0.5,0.1,0.5])
    camera.set_pose(pose)

    cnt = 0
    while True:
        p.stepSimulation()
        # camera.get_image()
        # img = np.copy(data[2][..., [2,1,0,3]])
        # mask = data[4]==data[4][240,220]
        # cv2.imshow('image', img)
        # cv2.waitKey(1)
        print(cnt)
        cnt+=1