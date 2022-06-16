import csv
import numpy as np
#import matplotlib.pyplot as plt
from pyomo.opt import SolverFactory, SolverManagerFactory
from controllers import price_model, MPC_model

class Simulation:
    def __init__(self,network,controller,results_filepath='results/sim/'):
        self.nw = network
        self.control = None
        self.path = results_filepath
        
    def initialize_devices():
        # we need like an activity log for all of the devices
        pass
    
    def update_device_actions():
        # Each device makes its charging decisions
        for device in self.nw.devices:
            if device.active == False:
                continue
            if device.opt is False:
                if device.node.prices[0] <= device.c_thres:
                    device.x = 1
            else:
                mpc = MPC_model(device)
                opt = SolverFactory('cplex_direct',solver_io="python")
                results=opt.solve(controller,tee=True)
    
    def update_device_requirements(self):
        for device in self.nw.devices:
            pass
        
    def timestep():
        pass
    
    def create_results_file(self):
        f = open(self.path+'.txt','w')
        headers = 't, '
        for i in list(self.nw.nodes.keys):
            headers += 'price'+str(i)+', '
        for j in list(self.nw.devices.keys):
            headers += 'device'+str(j[0])+str('_')+str(j[1])+', '
        headers = headers[:-2]+'\n'
        f.write(headers)
        f.close()
    
    def write_results(self,timestep):
        newline = 'timestep, '
        for i in list(self.nw.nodes.keys):
            newline += str(self.control.c[i,0])+', '
        newline = newline[:-2]+'\n'
        f = open(self.path+'.txt','a')
        f.write(newline)
        f.close()
    
    def update_prices(self):
        controller = price_model(self.nw)
        opt = SolverFactory('cplex_direct',solver_io="python")
        results=opt.solve(controller,tee=True)
        # pass down prices to nodes
        for i in self.nw.nodes:
            self.nw.nodes[i].prices = []# NEW PRICES
        
    
    def update_actions():
        pass
    
    
    