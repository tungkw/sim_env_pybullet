import pkgutil
import time

import cv2
import pybullet as p
import pybullet_data
# from pybullet_rendering import RenderingPlugin
# from pybullet_rendering.render.pyrender import PyrRenderer # pyrender-based renderer
from scipy.spatial.transform import Rotation as R
import numpy as np
from camera import Camera
from realsense import Realsense
from ur5 import UR5
import os
# os.environ['PYOPENGL_PLATFORM'] = 'egl'
# os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
# os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '330'
# os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.setGravity(0,0,-10)
    # client = p.connect(p.SHARED_MEMORY)
    # client = p.connect(p.SHARED_MEMORY_GUI)
    # client = p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(enableFileCaching=False)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, True)
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, True)
    print("numpy enabled", p.isNumpyEnabled())

    # egl = pkgutil.get_loader('eglRenderer')
    # if egl:
    #     print(egl.get_filename())
    #     plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
    #     print(plugin)


    # # renderer = PyrRenderer(multisamples=4) # or PyrRenderer(platform='egl', egl_device=1)
    # renderer = PyrRenderer(platform='egl', device_id=1)
    # plugin = RenderingPlugin(client, renderer)
    # print(plugin)

    # camera = Camera(fov=0)
    camera = Realsense()
    plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0,0,-2])
    pose = np.eye(4)
    pose[:3, :3] = R.from_euler('xyz', [210, 0, 0], degrees=True).as_matrix()
    pose[:3, 3] = np.array([0,-0.5,1])
    camera.set_pose(pose)

    arm = UR5()
    print(arm.get_joint_states()[0])
    arm.set_joint_target_positions([0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0])


    while True:
        p.stepSimulation()
        ts = time.time()
        data = camera.get_image()
        print(time.time() - ts)
        img = np.copy(data[0][..., [2,1,0,3]])
        depth = data[1]
        cv2.imshow('image', img)
        cv2.imshow('depth', (depth * 255).astype(np.uint8))
        cv2.waitKey(1)