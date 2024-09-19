import matplotlib.pyplot as plt
import numpy as np

# f = open("joints.txt", "r")
data = np.genfromtxt("build/joints.txt", delimiter="\t", dtype=float)

data = np.insert(data,0,[0,0,0],axis=1)

# data = np.reshape(data,(8,3))
print(data)


fig = plt.figure()

ax = plt.axes(projection='3d')
plt.axis('equal')

ax.set_xlim3d([-1000, 1000])
ax.set_ylim3d([-1000, 1000])
ax.set_zlim3d([0, 1300])
ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.plot3D(data[0], data[1], data[2], 'red')

ax.scatter3D(data[0], data[1], data[2])

plt.show()