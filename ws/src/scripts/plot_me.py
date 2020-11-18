import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def get_nerded(file_name):
    array = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=0)
    array = array.reshape((array.shape[0], 4, 4))
    return array

array1 = get_nerded("rrt_way.csv")
array2 = get_nerded("rrt_linear.csv")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(array1[:,0,3], array1[:,1,3], array1[:,2,3])
ax.scatter(array2[:,0,3], array2[:,1,3], array2[:,2,3])

plt.show()