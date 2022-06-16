#import csv
import numpy as np
from pyomo.opt import SolverFactory, SolverManagerFactory
from controllers import MPC_model
import datetime
#import matplotlib.pyplot as plt


# This wil create an object of a smart device associated at a building

class DistNetwork:
    def __init__(self,n_bus,lmps,S,sub,t_step=0.083,n_t=12*24):
        if len(lmps) < n_t:
            raise Exception('Insufficient LMPs')
        if len(sub) < n_bus:
            raise Exception('Insufficient Substation limits')
        self.n_bus = n_bus
        self.lmps = lmps
        self.t_step = t_step
        self.S = S
        self.n_t = n_t
        self.t_step = t_step
        self.nodes = {}
        self.devices = {}
        for i in range(n_bus):
            self.nodes[i] = Node(i,sub[i],t_step=t_step,n_t=n_t)
        self.loaded_bdgs = {}
    
    def add_household(self,node,bid='10017',startdate=datetime.datetime(2018,1,1)):
        if bid in self.loaded_bdgs:
            
        with open('data/bid_'+bid+'.csv','r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
    
    def add_EV(self,node,name,activity):
        self.devices[node,name] = EVCharger((node,name), activity,
                                            self.t_step,self.n_t)
            
    
class Controller:
    def __init__(self,network):
        self.t_step = network.t_step
        self.n_t = network.n_t
        
        
class Node:
    def __init__(self,name,P,t_step,n_t):
        self.id = name
        self.P = P # kW limit
        self.t_step = t_step
        self.n_t = n_t
        self.prices = [0.]*n_t
        self.d = None
        

class Device:
    def __init__(self,name,activity,t_step,n_t,p,E_est,choice='thres',thres=0):
        '''
        name (str): identifier of individual building
        t_step (float): size of timestep (hours)
        n_t (int): number of timesteps in the simulation
        deadline (int): number of timesteps to completion
        '''
        self.id = name
        self.node = name[0]
        self.active = False
        self.t_step = t_step
        self.n_t = n_t
        self.E_est = E_est# Estimated energy requirement over time_horizon
        self.E = 0#actual energy requirement (unknown by controller)
        self.p = p# Power of device
        
        # initialisation
        self.deadline = n_t #
        self.c_thres = np.inf# the threshold price at which the vehicle will turn on
        self.x = 0#is the device drawing power
        self.opt = False
        self.load_activity_log(activity)
        
        if choice == 'econ':
            self.consumer_choice_econ()
        elif choice == 'now':
            self.consumer_choice_now()
        elif choice == 'thres':
            self.consumer_choice_thres(thres)
        else:
            raise Exception('value for choice not recognized')
            
    def consumer_choice_econ(self):
        # The consumer chooses the economy setting
        self.opt = True
        
    def consumer_choice_now(self):
        # The consumer chooses the now setting
        self.c_thres = np.inf
        
    def consumer_choice_thres(self,c_thres):
        # The consumer chooses the price threshold setting
        self.c_thres = c_thres
        
    def is_charge(self):
        if self.opt == False:
            if self.node.prices[0] <= self.c_thres:
                return True
        
        # otherwise perform MPC
        mpc = MPC_model(self)
        
    def load_activity_log(self,path=''):
        self.events = []
        with open(path,'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                # start, end, energy
                self.events.append([int(row[0]),int(row[1]),float(row[2])])
        self.events.append([np.inf,np.inf,0.])
        
        if self.events[0][0] == 0:
            self.E = self.events[0][2]
            self.deadline = self.events[0][1]
        
    def step():
        # update times of events
        for i in range(len(self.events)):
            self.events[i][0] -= 1
            self.events[i][1] -= 1
        
        # if charging update current charge
        self.E -= self.p*self.x*self.t_step
        
        # if fully charged, plug out and remove event from log
        if self.E < 0:
            self.E = 0
            self.active = False
            self.events.pop(0)
            
        # if plugging in, activate
        if self.events[0][0] == 0:
            self.active = True
                                         

class EVCharger(Device):
    def __init__(self,name,activity,t_step,n_t):
        super().__init__(name, activity, t_step, n_t, 7.0, 20.0)
                                         
        
# In the pyomo model we will have variables for all smart devices, but we will fix them to off when not plugged in

# other option: directly CPLEX




#model.OBJ = pyo.Objective(expr = 2*model.x[1] + 3*model.x[2])
#model.Constraint1 = pyo.Constraint(expr = 3*model.x[1] + 4*model.x[2] >= 1)


        
        