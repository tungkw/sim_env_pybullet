import numpy as np
import pybullet as p
import pybullet_data
import time

vertices = []
faces = []
cells = []

angle = np.pi/6
r1 = 0.005
r2 = 0.01
t = 0.002
h = (r2-r1)/np.tan(angle)
sample_num = 20

outer = []
inner = []
for i in range(sample_num):
    x = r1 * np.cos(i/sample_num*np.pi*2)
    y = r1 * np.sin(i/sample_num*np.pi*2)
    outer.append([x, y, 0])
    x = (r1-t) * np.cos(i/sample_num*np.pi*2)
    y = (r1-t) * np.sin(i/sample_num*np.pi*2)
    inner.append([x, y, 0])
vertices.extend(inner)
vertices.extend(outer)
outer = np.array(outer)
inner = np.array(inner)

for j in range(sample_num):
    h_ = (j+1)/sample_num*h
    rate = (r1 + np.tan(angle) * h_) / r1
    cur_inner = inner * rate
    cur_outer = outer * rate
    cur_inner[:, 2] = h_
    cur_outer[:, 2] = h_
    vertices.extend(cur_inner.tolist())
    vertices.extend(cur_outer.tolist())
    for i in range(sample_num):
        tl_ = i + j*2*sample_num
        tr_ = (i+1)%sample_num + j*2*sample_num
        tl, tr = tl_+sample_num, tr_+sample_num
        bl_, br_ = tl_+2*sample_num, tr_+2*sample_num
        bl, br = tl_+3*sample_num, tr_+3*sample_num
        # outside
        faces.append([tl+1, tr+1, bl+1])
        faces.append([tr+1, br+1, bl+1])
        # inside
        faces.append([tl_+1, bl_+1, br_+1])
        faces.append([tl_+1, br_+1, tr_+1])
f = open("cup.obj", 'w')
print("o cup", file=f)
for vertice in vertices:
    print("v %f %f %f"%(vertice[0], vertice[1], vertice[2]), file=f)
print("s off", file=f)
for face in faces:
    print("f %d %d %d"%(face[0], face[1], face[2]), file=f)
f.close()

# print("s 1", file=f)
# for i in range(sample_num):
#     tl, tr = i, (i+1)%sample_num
#     bl, br = tl+sample_num, tr+sample_num
#     tl_, tr_ = tl+2*sample_num, tr+2*sample_num
#     bl_, br_ = tl+3*sample_num, tr+3*sample_num
#     # break
#     # top
#     print("f %d %d %d" % (tl_+1, tr_+1, tl+1), file=f)
#     print("f %d %d %d" % (tr_+1, tr+1, tl+1), file=f)
#     # bot
#     print("f %d %d %d" % (bl_+1, bl+1, br_+1), file=f)
#     print("f %d %d %d" % (br_+1, bl+1, br+1), file=f)
# f.close()
#
#
# f = open("cup.vtk", 'w')
# print("# vtk DataFile Version 2.0", file=f)
# print("cup, created by tungkw", file=f)
# print("ASCII", file=f)
# print("DATASET UNSTRUCTURED_GRID", file=f)
# print("POINTS 1036 double", file=f)
# for i in range(sample_num):
#     print("v %f %f %f" % (top[i][0], top[i][1], top[i][2]), file=f)
# for i in range(sample_num):
#     print("v %f %f %f" % (bot[i][0], bot[i][1], bot[i][2]), file=f)
# for i in range(sample_num):
#     print("v %f %f %f" % (top_[i][0], top_[i][1], top_[i][2]), file=f)
# for i in range(sample_num):
#     print("v %f %f %f" % (bot_[i][0], bot_[i][1], bot_[i][2]), file=f)
#
# print("CELLS {} {}".format(4436, 22180), file=f)
# print("CELL_TYPES {}".format(4436), file=f)
# for i in range(sample_num):
#     tl, tr = i, (i+1)%sample_num
#     bl, br = tl+sample_num, tr+sample_num
#     tl_, tr_ = tl+2*sample_num, tr+2*sample_num
#     bl_, br_ = tl+3*sample_num, tr+3*sample_num
#     # # outside
#     print("f %d %d %d" % (tl+1, tr+1, bl+1), file=f)
#     print("f %d %d %d" % (tr+1, br+1, bl+1), file=f)
#     # inside
#     print("f %d %d %d" % (tl_+1, bl_+1, br_+1), file=f)
#     print("f %d %d %d" % (tl_+1, br_+1, tr_+1), file=f)
#     # break
#     # top
#     print("f %d %d %d" % (tl_+1, tr_+1, tl+1), file=f)
#     print("f %d %d %d" % (tr_+1, tr+1, tl+1), file=f)
#     # bot
#     print("f %d %d %d" % (bl_+1, bl+1, br_+1), file=f)
#     print("f %d %d %d" % (br_+1, bl+1, br+1), file=f)
# f.close()





p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.resetDebugVisualizerCamera(3, -420, -30, [0.3, 0.9, -2])
# p.setGravity(0,0,-10)

p.loadURDF("plane.urdf", [0,0,-2])
# oid = p.loadSoftBody(
#     fileName='cup.obj',
#     basePosition=[0,0,-2],
#     scale=10,
#     mass=0.1,
#     useNeoHookean = True,
#     # useBendingSprings=1,
#     useMassSpring=1,
#     springElasticStiffness=100,
#     springDampingStiffness=.1,
#     springDampingAllDirections = 1,
#     # useSelfCollision = 1,
#     frictionCoeff = 10,
#     useFaceContact=1,
#     # repulsionStiffness=800,
#     collisionMargin=0.006,
# )

vs = p.createVisualShape(shapeType=p.GEOM_MESH, fileName='cup.obj', meshScale=[10,10,10])
oid = p.createMultiBody(baseVisualShapeIndex=vs)

# p.changeVisualShape(oid, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

while True:
    p.stepSimulation()
    # time.sleep(0.05)