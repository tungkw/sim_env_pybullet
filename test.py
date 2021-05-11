import pkgutil

import cv2
import pybullet as p
import pybullet_data
# from pybullet_rendering import RenderingPlugin
# from pybullet_rendering.render.pyrender import PyrRenderer # pyrender-based renderer
from scipy.spatial.transform import Rotation as R
import numpy as np
from camera import Camera
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

    # camera = Camera()
    # plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), [0,0,-2])
    # pose = np.eye(4)
    # pose[:3, :3] = R.from_euler('xyz', [210, 0, 0], degrees=True).as_matrix()
    # pose[:3, 3] = np.array([0,-0.5,1])
    # camera.set_pose(pose)

    # arm = UR5()
    # print(arm.get_joint_positions())
    # arm.set_joint_target_positions([0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0])

    # ball = p.loadSoftBody(
    #     fileName="/home/tungkw/.local/lib/python3.6/site-packages/pybullet_data/sphere_smooth.obj",
    #     scale=0.2,
    #     basePosition=[0,0,0],
    #     mass=5,
    #     collisionMargin = 0.006,
    #
    #     useMassSpring=True,
    #     springElasticStiffness=40,
    #     springDampingStiffness=.1,
    #     springDampingAllDirections=1,
    #
    #     useBendingSprings=True,
    #     # springBendingStiffness=1,
    #
    #     # useNeoHookean=False,
    #     # NeoHookeanMu=1500000,
    #     # NeoHookeanLambda=600,
    #     # NeoHookeanDamping=1e-10,
    #
    #     frictionCoeff = 0.5,
    #     useFaceContact=True,
    #     # useSelfCollision = 1,
    #     # repulsionStiffness=1000,
    # )
    # p.changeVisualShape(ball, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    # bunnyId = p.loadSoftBody(fileName="/home/tungkw/.local/lib/python3.6/site-packages/pybullet_data/torus/torus_textured.obj",
    #                          mass=3, useNeoHookean=1,
    #                          NeoHookeanMu=180, NeoHookeanLambda=600, NeoHookeanDamping=0.01, collisionMargin=0.006,
    #                          # useSelfCollision=1,
    #                          frictionCoeff=0.5, repulsionStiffness=800)
    # p.changeVisualShape(bunnyId, -1, rgbaColor=[1, 1, 1, 1])
    # p.changeVisualShape(bunnyId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    tex = p.loadTexture("uvmap.png")
    planeId = p.loadURDF("plane.urdf", [0, 0, -2])

    boxId = p.loadURDF("cube.urdf", [0, 3, 2], useMaximalCoordinates=True)

    bunnyId = p.loadSoftBody("torus/torus_textured.obj", mass=3, useNeoHookean=1,
                             NeoHookeanMu=180, NeoHookeanLambda=600, NeoHookeanDamping=0.01, collisionMargin=0.006,
                             useSelfCollision=1, frictionCoeff=0.5, repulsionStiffness=800)
    p.changeVisualShape(bunnyId, -1, rgbaColor=[1, 1, 1, 1], textureUniqueId=tex, flags=0)


    while True:
        p.stepSimulation()
        # data = camera.get_image()
        # img = np.copy(data[2][..., [2,1,0,3]])
        # mask = data[4]==data[4][240,220]
        # cv2.imshow('image', img)
        # cv2.waitKey(1)