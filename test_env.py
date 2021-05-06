from UR5CamGymEnv import Ur5CamGymEnv

if __name__ == '__main__':
    env = Ur5CamGymEnv(renders=True)

    motorsIds = []
    # motorsIds.append(environment._p.addUserDebugParameter("posX",0.4,0.75,0.537))
    # motorsIds.append(environment._p.addUserDebugParameter("posY",-.22,.3,0.0))
    # motorsIds.append(environment._p.addUserDebugParameter("posZ",0.1,1,0.2))
    # motorsIds.append(environment._p.addUserDebugParameter("yaw",-3.14,3.14,0))
    # motorsIds.append(environment._p.addUserDebugParameter("fingerAngle",0,0.3,.3))

    dv = 1
    motorsIds.append(env._p.addUserDebugParameter("posX", -dv, dv, 0))
    motorsIds.append(env._p.addUserDebugParameter("posY", -dv, dv, 0))
    motorsIds.append(env._p.addUserDebugParameter("posZ", -dv, dv, 0))
    motorsIds.append(env._p.addUserDebugParameter("yaw", -dv, dv, 0))
    motorsIds.append(env._p.addUserDebugParameter("fingerAngle", 0, 0.3, .3))

    state = env.reset()
    while True:
        action = []
        for motorId in motorsIds:
            action.append(env._p.readUserDebugParameter(motorId))
        o, r, done, _ = env.step(action)
        if done:
            break