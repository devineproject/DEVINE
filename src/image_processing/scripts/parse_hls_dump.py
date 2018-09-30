#!/usr/bin/env python2
'''Simple parser of HLSDump.txt + graph of the results'''
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

import re

if __name__ == '__main__':
    with open('HLSDump.txt', 'r') as f:
        data = f.read()
        values = re.findall('\[\ ?(\d+) (\d+) (\d+)\]', data)
        hues = []
        lightness = []
        saturations = []
        for value in values:
            [h, l, s] = value
            hues.append(int(h))
            lightness.append(int(l))
            saturations.append(int(s))

        gs = gridspec.GridSpec(2, 2)
        fig = plt.figure()

        graph1 = fig.add_subplot(gs[0, 0])
        graph1.scatter(hues, saturations)
        graph1.set_xlabel("Hues")
        graph1.set_ylabel("Saturations")

        graph2 = fig.add_subplot(gs[0, 1])
        graph2.scatter(hues, lightness)
        graph2.set_xlabel("Hues")
        graph2.set_ylabel("Lightness")

        graph3 = fig.add_subplot(gs[1, :], projection='3d')
        graph3.scatter(hues, saturations, lightness)
        graph3.set_xlabel("Hues")
        graph3.set_ylabel("Saturations")
        graph3.set_zlabel("Lightness")

        fig.show()

    raw_input('Press Enter to exit')
