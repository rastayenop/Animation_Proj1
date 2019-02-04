#!/usr/local/bin-python3/python

import csv

import PyQt5
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

xs = []
ys = []
zs = []

with open('skel_pos') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
         xs.append(np.float(row[1]))
         ys.append(np.float(row[2]))
         zs.append(np.float(row[3]))

fig = plt.figure()
ax = Axes3D(fig)

#ax = fig.add_subplot(111, projection='3d')
ax.scatter(xs, ys, zs, zdir='z', s=20)

plt.show()
