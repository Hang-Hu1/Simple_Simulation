import sys
sys.path.append('../')

from pathlib import Path
import numpy as np
from importlib import import_module
import scipy.optimize
import time
import matplotlib.pyplot as plt
from tqdm import tqdm
import pickle
import os

from py_rigid_body_simulation.core.py_rigid_body_simulation_core import Vec3, MassSpring

if __name__ == '__main__':

    system = MassSpring(2, 1)
    time_step = 0.1
    xlist = []
    ylist = []
    zlist = []
    tlist = []
    for i in range(400):
        xlist.append(system.position(0).value(0))
        ylist.append(system.position(0).value(1))
        zlist.append(system.position(0).value(2))
        tlist.append(i * time_step)
        system.step_forward(time_step, 10)

    plt.subplot(3, 1, 1)
    plt.plot(tlist, xlist)
    plt.subplot(3, 1, 2)
    plt.plot(tlist, ylist)
    plt.subplot(3, 1, 3)
    plt.plot(tlist, zlist)
    plt.show()