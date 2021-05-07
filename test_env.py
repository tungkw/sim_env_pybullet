from UR5CamGymEnv import Ur5CamGymEnv

if __name__ == '__main__':
    env = Ur5CamGymEnv(renders=True)

    motorsIds = []
    dv = 1
    motorsIds.append(env._p.addUserDebugParameter("posX", -dv, dv, 0))
    motorsIds.append(env._p.addUserDebugParameter("posY", -dv, dv, 0))
    motorsIds.append(env._p.addUserDebugParameter("yaw", -dv, dv, 0))

    state = env.reset()
    while True:
        action = []
        for motorId in motorsIds:
            action.append(env._p.readUserDebugParameter(motorId))
        o, r, done, _ = env.step(action)
        if done:
            break