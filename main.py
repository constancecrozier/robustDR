import csv, datetime
#import numpy as np
#import matplotlib.pyplot as plt

from components import (DistNetwork, EVCharger)
from sim_platform import Simulation

# create a 1 bus distribution network at transmission node #?
network = DistNetwork(1,[1.]*(12*24),60.,[20.]*3)

# add households to each node
for i in range(network.n_bus):
    for n in range(1):
        network.add_household(i,'data/bid_10017.csv',
                              datetime.datetime(2018,1,1))

# add EV chargers to each node
for i in range(network.n_bus):
    for n in range(1):
        network.add_EV(i,n)

# intialize simulation
sim = Simulation(network)
sim.update_prices()