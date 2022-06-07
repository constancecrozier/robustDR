import csv
#import numpy as np
#import matplotlib.pyplot as plt

from components import (DistNetwork, EVCharger)
from sim_platform import Simulation
from controller import opt_model

# create a 2 bus distribution network at transmission node #?
network = DistNetwork(1,[1.]*(12*24),60.,[20.]*3)

# add households to each node
for i in range(network.n_bus):
    for n in range(2):
        network.add_household(i)

# add EV chargers to each node
for i in range(network.n_bus):
    for n in range(2):
        network.add_EV(i,n)

# set up controller 
controller = opt_model(network)

# intialize simulation
sim = Simulation(network,controller)
sim.update_prices()