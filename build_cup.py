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
sample_layer = 10

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

for j in range(sample_layer):
    h_ = (j+1)/sample_layer*h
    rate = (r1 + np.tan(angle) * h_) / r1
    cur_inner = inner * rate
    cur_outer = outer * rate
    cur_inner[:, 2] = h_
    cur_outer[:, 2] = h_
    vertices.extend(cur_inner.tolist())
    vertices.extend(cur_outer.tolist())
    for i in range(sample_num):
        tl_, tr_ = i + j*2*sample_num, (i+1)%sample_num + j*2*sample_num
        tl, tr = tl_+sample_num, tr_+sample_num
        bl_, br_ = tl_+2*sample_num, tr_+2*sample_num
        bl, br = tl_+3*sample_num, tr_+3*sample_num
        # outside
        faces.append([tl+1, tr+1, bl+1])
        faces.append([tr+1, br+1, bl+1])
        # inside
        faces.append([tl_+1, bl_+1, br_+1])
        faces.append([tl_+1, br_+1, tr_+1])
        # cell
        # if j==0 and i ==0:
        cells.append([tl_, tr_, tl, bl_])
        cells.append([tr_, tr, tl, br])
        cells.append([tr_, tl, bl_, bl])
        cells.append([tr_, br, tl, bl])
        cells.append([tr_, bl_, br_, bl])
        cells.append([tr_, br_, br, bl])
        cells.append([tr_, br_, br, bl])


        # cells.append([tl_, tr_, tr, tl, bl_, br_, br, bl])

        # cells.append([tl+1, tr+1, bl+1])
        # cells.append([tr+1, br+1, bl+1])
        # # inside
        # cells.append([tl_+1, bl_+1, br_+1])
        # cells.append([tl_+1, br_+1, tr_+1])
for i in range(sample_num):
    tl_, tr_ = i, (i+1)%sample_num
    tl, tr = tl_+sample_num, tr_+sample_num
    bl_, br_ = tl_+(2*(sample_layer+1)-2)*sample_num, tr_+(2*(sample_layer+1)-2)*sample_num
    bl, br = tl_+(2*(sample_layer+1)-1)*sample_num, tr_+(2*(sample_layer+1)-1)*sample_num
    # top
    faces.append([tl_+1, tr_+1, tl+1])
    faces.append([tr_+1, tr+1, tl+1])
    # bot
    faces.append([bl_+1, bl+1, br_+1])
    faces.append([br_+1, bl+1, br+1])
    # # cell
    # # top
    # cells.append([tl_+1, tr_+1, tl+1])
    # cells.append([tr_+1, tr+1, tl+1])
    # # bot
    # cells.append([bl_+1, bl+1, br_+1])
    # cells.append([br_+1, bl+1, br+1])

f = open("cup.obj", 'w')
print("o cup", file=f)
for vertice in vertices:
    print("v %f %f %f"%(vertice[0], vertice[1], vertice[2]), file=f)
print("s off", file=f)
for face in faces:
    print("f %d %d %d"%(face[0], face[1], face[2]), file=f)
f.close()



f = open("cup.vtk", 'w')
print("# vtk DataFile Version 2.0", file=f)
print("cup, created by tungkw", file=f)
print("ASCII", file=f)
print("DATASET UNSTRUCTURED_GRID", file=f)
print("POINTS {} double".format(len(vertices)), file=f)
for vertice in vertices:
    print("%f %f %f"%(vertice[0], vertice[1], vertice[2]), file=f)
print(file=f)
# print("CELLS {} {}".format(len(cells), len(cells)*9), file=f)
print("CELLS {} {}".format(len(cells), len(cells)*5), file=f)
# print("CELLS {} {}".format(len(cells), len(cells)*(1+4)), file=f)
for cell in cells:
    # print("8 %d %d %d %d %d %d %d %d"%(cell[0], cell[1], cell[2], cell[3], cell[4], cell[5], cell[6], cell[7]), file=f)
    print("4 %d %d %d %d"%(cell[0], cell[1], cell[2], cell[3]), file=f)
    # print("3 %d %d %d"%(cell[0], cell[1], cell[2]), file=f)
print(file=f)
print("CELL_TYPES {}".format(len(cells)), file=f)
for cell in cells:
    # print("12", file=f)
    print("10", file=f)
    # print("5", file=f)
f.close()


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.resetDebugVisualizerCamera(3, -420, -30, [0.3, 0.9, -2])
p.setGravity(0,0,-10)

p.loadURDF("plane.urdf", [0,0,-2])

# oid = p.loadSoftBody(
#     fileName="cup.obj",
#     # simFileName="cup.vtk",
#     basePosition=[0,0,-1.5],
#     scale=1,
#     mass=1,
#     useNeoHookean=0,
#     useBendingSprings=1,
#     useMassSpring=1,
#     springElasticStiffness=400,
#     springDampingStiffness=.1,
#     springDampingAllDirections = 1,
#     useSelfCollision = 1,
#     frictionCoeff = 1,
#     useFaceContact=1,
#     # repulsionStiffness=800,
#     collisionMargin=0.006,
# )
oid = p.loadSoftBody(
    "cup.obj",
    basePosition = [0,0,-1.5],
    scale = 10,
    mass = 1.,
    useNeoHookean = 0,
    useBendingSprings=1,
    useMassSpring=1,
    springElasticStiffness=1,
    springDampingStiffness=.5,
    springDampingAllDirections = 1,
    useSelfCollision = 1,
    frictionCoeff = .5,
    useFaceContact=1
)
# oid = p.loadSoftBody(
#     fileName="cup.obj",
#     # simFileName="cup.obj",
#     basePosition=[0,0,-2],
#     scale=10,
#     mass=0.1,
#     useNeoHookean=1,
#     NeoHookeanMu=100,
#     NeoHookeanLambda=6000,
#     NeoHookeanDamping=100,
#     collisionMargin=0.0006,
#     useSelfCollision=1,
#     frictionCoeff=0.5,
#     repulsionStiffness=800,
# )
# oid = p.loadURDF("cup.urdf", [0,0,-2])

# vs = p.createVisualShape(shapeType=p.GEOM_MESH, fileName='cup.obj', meshScale=[10,10,10])
# oid = p.createMultiBody(baseVisualShapeIndex=vs)

# p.changeVisualShape(oid, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

while True:
    p.stepSimulation()
    # time.sleep(0.05)