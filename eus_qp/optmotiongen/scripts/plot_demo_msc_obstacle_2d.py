#!/usr/bin/env python

import numpy as np
import rospkg
import matplotlib.pyplot as plt

rospack = rospkg.RosPack()


def plot(filename="config-msc.csv"):
    filename_full = rospack.get_path("eus_qp")+"/optmotiongen/logs/demo-msc-obstacle-2d/"+filename
    plt.close('all')
    with open(filename_full) as f:
        lines = (line for line in f if not line.startswith('%'))
        data = np.loadtxt(lines, delimiter=',')

        fig = plt.figure()
        cmap = plt.get_cmap('tab20')
        solution_num = int(data.shape[1] / 2)
        for i in range(data.shape[0]-1):
            for j in range(solution_num):
                plt.plot(data[i:i+2,2*j], data[i:i+2,2*j+1], 'o-', color=cmap(j))
        fig.show()
