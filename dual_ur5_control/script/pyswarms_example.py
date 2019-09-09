# Import modules
import numpy as np
import matplotlib.pyplot as plt
import math

# Import PySwarms
import pyswarms as ps
from pyswarms.utils.functions import single_obj as fx


# create a parameterized version of the classic Rosenbrock unconstrained optimzation function
def get_a_cost_val(x):

    f = float(input("Input a cost val"))
    #import pdb
    #pdb.set_trace()
    return f


# Set-up hyperparameters as dict
options = {'c1': 0.5, 'c2': 0.3, 'w':0.9}

# set bounds
g_max = np.array([0.6, 0.6, 0.6, math.pi, math.pi, math.pi])
g_min = np.array([-0.6, 0.3, 0.25, -math.pi, -math.pi, -math.pi])
bounds = (g_min, g_max)

# Create an instance of PSO optimizer
optimizer = ps.single.GlobalBestPSO(n_particles=10, dimensions=6, options=options) # global best PSO

# Perform optimization: call the optimize() and store the optimal cost as well as positions
cost, pos = optimizer.optimize(get_a_cost_val, iters=10)



