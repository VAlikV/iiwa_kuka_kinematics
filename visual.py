import matplotlib.pyplot as plt
import numpy as np

a = np.array([-11.26, 8.749, 12.28, 112.3, 168, -8.938, 80.49])
print(a*np.pi/180)

# f = open("joints.txt", "r")
data = np.genfromtxt("joints.txt", delimiter="\t", dtype=float).T
data1 = np.genfromtxt("joints1.txt", delimiter="\t", dtype=float).T

data = np.insert(data,0,[0,0,0],axis=1)
data1 = np.insert(data1,0,[0,0,0],axis=1)

# data = np.reshape(data,(8,3))
print(data.T)
print("------------------------------------------")
print(data1.T)


fig = plt.figure()

ax = plt.axes(projection='3d')
plt.axis('equal')

ax.set_xlim3d([-1, 1])
ax.set_ylim3d([-1, 1])
ax.set_zlim3d([0, 1.3])
ax.set_xlabel("X")
ax.set_ylabel("Y")

ax.plot3D(data[0], data[1], data[2], 'red')
ax.plot3D(data1[0], data1[1], data1[2], 'green')

ax.scatter3D(data[0], data[1], data[2])
ax.scatter3D(data1[0], data1[1], data1[2], 'blue')


plt.show()