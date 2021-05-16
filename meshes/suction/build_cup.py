import numpy as np
import os

vertices = []
faces = []
cells = []

angle = np.pi / 4
r1 = 0.01
r2 = 0.015
t = 0.001
h = (r2 - r1) / np.tan(angle)
sample_num = 20
sample_layer = 1

outer = []
inner = []
for i in range(sample_num):
    x = (r1 - t) * np.cos(i / sample_num * np.pi * 2)
    y = (r1 - t) * np.sin(i / sample_num * np.pi * 2)
    inner.append([x, y, 0])
    x = r1 * np.cos(i / sample_num * np.pi * 2)
    y = r1 * np.sin(i / sample_num * np.pi * 2)
    outer.append([x, y, 0])
vertices.extend(inner)
vertices.extend(outer)
inner = np.array(inner)
outer = np.array(outer)

for j in range(sample_layer):
    h_ = (j + 1) / sample_layer * h
    rate = (r1 + np.tan(angle) * h_) / r1
    cur_inner = inner * rate
    cur_outer = outer * rate
    cur_inner[:, 2] = h_
    cur_outer[:, 2] = h_
    vertices.extend(cur_inner.tolist())
    vertices.extend(cur_outer.tolist())
    for i in range(sample_num):
        l1, l0 = i + j * 2 * sample_num, (i + 1) % sample_num + j * 2 * sample_num
        l2, l3 = l1 + sample_num, l0 + sample_num
        l5, l4 = l1 + 2 * sample_num, l0 + 2 * sample_num
        l6, l7 = l1 + 3 * sample_num, l0 + 3 * sample_num

        # outside
        faces.append([l2 + 1, l3 + 1, l7 + 1])
        faces.append([l7 + 1, l6 + 1, l2 + 1])
        # inside
        faces.append([l0 + 1, l1 + 1, l5 + 1])
        faces.append([l5 + 1, l4 + 1, l0 + 1])
        # cell
        cells.append([l0, l1, l2, l5])
        cells.append([l0, l2, l3, l7])
        cells.append([l4, l7, l5, l0])
        cells.append([l5, l7, l6, l2])
        cells.append([l0, l5, l2, l7])

for i in range(sample_num):
    tl_, tr_ = i, (i + 1) % sample_num
    tl, tr = tl_ + sample_num, tr_ + sample_num
    bl_, br_ = tl_ + (2 * (sample_layer + 1) - 2) * sample_num, tr_ + (2 * (sample_layer + 1) - 2) * sample_num
    bl, br = tl_ + (2 * (sample_layer + 1) - 1) * sample_num, tr_ + (2 * (sample_layer + 1) - 1) * sample_num
    # top
    faces.append([tl_ + 1, tr_ + 1, tl + 1])
    faces.append([tr_ + 1, tr + 1, tl + 1])
    # bot
    faces.append([bl_ + 1, bl + 1, br_ + 1])
    faces.append([br_ + 1, bl + 1, br + 1])

f = open(os.path.join(os.path.dirname(__file__), "cup.obj"), 'w')
print("o cup", file=f)
for vertice in vertices:
    print("v %f %f %f" % (vertice[0], vertice[1], vertice[2]), file=f)
print("s off", file=f)
for face in faces:
    print("f %d %d %d" % (face[0], face[1], face[2]), file=f)
f.close()

f = open(os.path.join(os.path.dirname(__file__), "cup.vtk"), 'w')
print("# vtk DataFile Version 2.0", file=f)
print("cup, created by tungkw", file=f)
print("ASCII", file=f)
print("DATASET UNSTRUCTURED_GRID", file=f)
print("POINTS {} double".format(len(vertices)), file=f)
for vertice in vertices:
    print("%f %f %f" % (vertice[0], vertice[1], vertice[2]), file=f)
print(file=f)
print("CELLS {} {}".format(len(cells), len(cells) * 5), file=f)
for cell in cells:
    print("4 %d %d %d %d" % (cell[0], cell[1], cell[2], cell[3]), file=f)
print(file=f)
print("CELL_TYPES {}".format(len(cells)), file=f)
for cell in cells:
    print("10", file=f)
f.close()
