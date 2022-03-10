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

from py_rigid_body_simulation.core.py_rigid_body_simulation_core import HelloWorld, Vec3, FreeFaller
from py_rigid_body_simulation.common.common import print_info, print_ok, print_error, print_warning, ndarray
from py_rigid_body_simulation.common.grad_check import check_gradients
from py_rigid_body_simulation.common.display import export_gif

if __name__ == '__main__':
    greeter = HelloWorld()
    greeter.greet()

    x = Vec3(0, 10, 0)
    v = Vec3(1, 2, 0)
    time_step = 0.1
    xlist = []
    ylist = []
    zlist = []
    tlist = []
    ff = FreeFaller(1.0, x, v)
    for i in range(10):
        xlist.append(ff.position().value(0))
        ylist.append(ff.position().value(1))
        zlist.append(ff.position().value(2))
        tlist.append(i * time_step)
        ff.step(time_step, Vec3(0,0,0))

    plt.subplot(2, 2, 1)
    plt.plot(xlist, ylist)
    plt.subplot(2, 2, 2)
    plt.plot(tlist, xlist)
    plt.subplot(2, 2, 3)
    plt.plot(tlist, ylist)
    plt.subplot(2, 2, 4)
    plt.plot(tlist, zlist)
    plt.show()