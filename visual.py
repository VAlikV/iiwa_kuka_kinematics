import matplotlib.pyplot as plt
import numpy as np

# f = open("joints.txt", "r")
data = np.genfromtxt("build/joints.txt", delimiter="\t", dtype=float)
data1 = np.genfromtxt("build/joints1.txt", delimiter="\t", dtype=float)

data2 = np.genfromtxt("build/hui.txt", delimiter="\t", dtype=float)[:,:3]

data = np.insert(data,0,[0,0,0],axis=1)
data1 = np.insert(data1,0,[0,0,0],axis=1)

# data = np.reshape(data,(8,3))
print(data)
print("------------------------------------------")
print(data1)


fig = plt.figure("1")

ax = plt.axes(projection='3d')
plt.axis('equal')

ax.set_xlim3d([-1, 1])
ax.set_ylim3d([-1, 1])
ax.set_zlim3d([0, 1.3])
ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.plot3D(data[0], data[1], data[2], 'red')
ax.plot3D(data1[0], data1[1], data1[2], 'green')
ax.plot3D(data2.T[0], data2.T[1], data2.T[2], 'violet')


ax.scatter3D(data[0], data[1], data[2])
ax.scatter3D(data1[0], data1[1], data1[2], 'blue')


plt.show()