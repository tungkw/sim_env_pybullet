
import gym
from gym import spaces
from gym.utils import seeding

import pybullet as p
import pybullet_data

import numpy as np
import time
import random
import os
from pkg_resources import parse_version

from ur5 import UR5 as Arm
from camera import Camera

maxSteps = 1000

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


class Ur5CamGymEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
               urdfRoot=os.path.join(os.path.dirname(__file__), "meshes"),
               actionRepeat=1,
               isEnableSelfCollision=True,
               renders=False,
               isDiscrete=False):

        # pybullet setting
        self._timeStep = 1. / 240.
        self._urdfRoot = urdfRoot
        self._isEnableSelfCollision = isEnableSelfCollision
        self._p = p
        if renders:
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                p.connect(p.GUI)
                # p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
                # p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
        else:
            p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        #timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")

        # camera setting
        self._width = 341
        self._height = 256

        # env setting
        if isDiscrete:
          self.action_space = spaces.Discrete(7)
        else:
          action_dim = 3
          action_bound = 1
          self.action_space = spaces.Box(low=-action_bound, high=action_bound, shape=(action_dim,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0,
                                            high=255,
                                            shape=(self._height, self._width, 4),
                                            dtype=np.uint8)
        self._envStepCounter = 0
        self._renders = renders
        self._isDiscrete = isDiscrete
        self._actionRepeat = actionRepeat
        self._observation = []
        self.terminated = 0
        self.seed()
        # self.reset()


    def reset(self):
        self.terminated = 0
        self._envStepCounter = 0

        # reset pybullet
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setGravity(0, 0, -10)
        p.setTimeStep(self._timeStep)

        # scene
        p.loadURDF("plane.urdf", [0, 0, -1])
        p.loadURDF("table/table.urdf", 0.5000000, 0.00000, -.820000,
                   0.000000, 0.000000, 0.0, 1.0)
        xpos = 0.5 + 0.2 * random.random()
        ypos = 0 + 0.25 * random.random()
        ang = 3.1415925438 * random.random()
        orn = p.getQuaternionFromEuler([0, 0, ang])
        self.blockUid = p.loadURDF("block.urdf", xpos, ypos, -0.1,
                                   orn[0], orn[1], orn[2], orn[3])

        # robotic arm
        self.arm = Arm()
        self.init_position = np.array([0,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,0])
        self.arm.set_joint_target_positions(self.init_position)
        while np.linalg.norm(self.arm.get_joint_positions()-self.init_position) > 0.01:
            p.stepSimulation()

        # camera
        self.camera = Camera()

        p.stepSimulation()
        self._observation = self.getExtendedObservation()
        return np.array(self._observation)

    def __del__(self):
        p.disconnect()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def getExtendedObservation(self):
        img_arr = self.camera.get_image()
        rgb = img_arr[2]
        # np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
        self._observation = rgb
        return self._observation

    def step(self, action):
        if (self._isDiscrete):
          dv = 0.01
          dx = [0, -dv, dv, 0, 0, 0, 0][action]
          dy = [0, 0, 0, -dv, dv, 0, 0][action]
          da = [0, 0, 0, 0, 0, -0.1, 0.1][action]
          f = 0.3
          realAction = [dx, dy, -0.002, da, f]
        else:
          dv = 0.01
          dx = action[0] * dv
          dy = action[1] * dv
          da = action[2] * 0.1
          f = 0.3
          realAction = [dx, dy, -0.002, da, f]

        return self.step2(realAction)

    def step2(self, action):
        for i in range(self._actionRepeat):
          self._kuka.applyAction(action)
          p.stepSimulation()
          if self._termination():
            break
          #self._observation = self.getExtendedObservation()
          self._envStepCounter += 1

        self._observation = self.getExtendedObservation()
        if self._renders:
          time.sleep(self._timeStep)
        done = self._termination()
        reward = self._reward()
        return np.array(self._observation), reward, done, {}

    def render(self, mode='human', close=False):
        if mode != "rgb_array":
          return np.array([])
        return self.getExtendedObservation()

    def _termination(self):
        #print (self._kuka.endEffectorPos[2])
        state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)
        actualEndEffectorPos = state[0]

        #print("self._envStepCounter")
        #print(self._envStepCounter)
        if (self.terminated or self._envStepCounter > maxSteps):
          self._observation = self.getExtendedObservation()
          return True
        maxDist = 0.005
        closestPoints = p.getClosestPoints(self._kuka.trayUid, self._kuka.kukaUid, maxDist)

        if (len(closestPoints)):  #(actualEndEffectorPos[2] <= -0.43):
          self.terminated = 1

          #print("closing gripper, attempting grasp")
          #start grasp and terminate
          fingerAngle = 0.3
          for i in range(100):
            graspAction = [0, 0, 0.0001, 0, fingerAngle]
            self._kuka.applyAction(graspAction)
            p.stepSimulation()
            fingerAngle = fingerAngle - (0.3 / 100.)
            if (fingerAngle < 0):
              fingerAngle = 0

          for i in range(1000):
            graspAction = [0, 0, 0.001, 0, fingerAngle]
            self._kuka.applyAction(graspAction)
            p.stepSimulation()
            blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
            if (blockPos[2] > 0.23):
              #print("BLOCKPOS!")
              #print(blockPos[2])
              break
            state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)
            actualEndEffectorPos = state[0]
            if (actualEndEffectorPos[2] > 0.5):
              break

          self._observation = self.getExtendedObservation()
          return True
        return False

    def _reward(self):
        #rewards is height of target object
        blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
        closestPoints = p.getClosestPoints(self.blockUid, self._kuka.kukaUid, 1000, -1,
                                           self._kuka.kukaEndEffectorIndex)

        reward = -1000
        numPt = len(closestPoints)
        #print(numPt)
        if (numPt > 0):
          #print("reward:")
          reward = -closestPoints[0][8] * 10
        if (blockPos[2] > 0.2):
          #print("grasped a block!!!")
          #print("self._envStepCounter")
          #print(self._envStepCounter)
          reward = reward + 1000

        #print("reward")
        #print(reward)
        return reward

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step