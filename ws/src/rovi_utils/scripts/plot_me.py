#!/usr/bin/env python3

import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_nerded(file_name):
	array = np.loadtxt(open(file_name, "rb"), delimiter=",", skiprows=0)
	array = array.reshape((array.shape[0], 4, 4))
	return array

arr_traj_lin = get_nerded("traj_par.csv")
arr_traj_rrt = get_nerded("traj_rrt.csv")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(arr_traj_rrt[:,0,3], arr_traj_rrt[:,1,3], arr_traj_rrt[:,2,3])
ax.scatter(arr_traj_lin[:,0,3], arr_traj_lin[:,1,3], arr_traj_lin[:,2,3])

plt.show()