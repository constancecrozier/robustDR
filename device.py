import csv
import numpy as np
import matplotlib.pyplot as plt


# This wil create an object of a smart device associated at a building

class DistNetwork:
    def __init__(self,n_bus,lmps=[],t_step=0.083,n_t=12*24):
        if len(lmps) < n_t:
            raise Exception('Insufficient LMPs')
        self.n_bus = n_bus
        self.nodes = {}
        self.lmps = []
        for i in range(n_bus):
            self.nodes[i] = Node(i,t_step=t_step,n_t=n_t)
    
    def add_devices(self,filepath):
        devices = {}
        for i in self.nodes:
            self.nodes[i].devices.append(EVCharger((i,0),
                                                   t_step,n_t))
            devices[i,0] = self.nodes[i][-1]
        self.devices = devices
            
    
class Controller:
    def __init__(self,network):
        self.t_step = network.t_step
        self.n_t = network.n_t
        
        
class Node:
    def __init__(self,name,t_step,n_t):
        self.id = name
        self.t_step = t_step
        self.n_t = n_t
        self.devices = []
        self.prices = [0.]*n_t


class Device:
    def __init__(self,name,t_step,n_t,c_rate,d_rate):
        '''
        name (str): identifier of individual building
        t_step (float): size of timestep (hours)
        n_t (int): number of timesteps in the simulation
        c_rate (float): maximum charging rate (kW)
        d_rate (float): maximum discharging rate (kW)
        deadline (int): number of timesteps to completion
        '''
        self.id = name
        self.active = False
        self.c_rate = 
        self.d_rate = # maybe
        self.energy = 0# Energy requirement over time_horizon
        self.power = # Power of device
        
        # initialisation
        self.deadline = n_t #
        self.c_thres = np.inf# the threshold price at which the vehicle will turn on
        self.x = 0#is the device drawing power
        self.opt = False
        
    def consumer_choice_econ(self):
        # The consumer chooses the economy setting
        self.opt = True
        
    def consumer_choice_now(self):
        # The consumer chooses the now setting
        self.c_thres = np.inf
        
    def consumer_choice_thres(self,c_thres):
        # The consumer chooses the price threshold setting
        self.c_thres = c_thres
        
    def step():
        pass
                                         

class EVCharger(Device):
    def __init__(self,name,t_step,n_t):
        super().__init__(name, t_step, n_t, 7.0, 0.0)
                                         
        
# In the pyomo model we will have variables for all smart devices, but we will fix them to off when not plugged in

# other option: directly CPLEX




#model.OBJ = pyo.Objective(expr = 2*model.x[1] + 3*model.x[2])
#model.Constraint1 = pyo.Constraint(expr = 3*model.x[1] + 4*model.x[2] >= 1)


        
        