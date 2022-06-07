import csv
import numpy as np
#import matplotlib.pyplot as plt
from pyomo.opt import SolverFactory, SolverManagerFactory

class Simulation:
    def __init__(self,network,controller,results_filepath='results/sim'):
        self.nw = network
        self.control = controller
        self.path = results_filepath
        
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
        opt = SolverFactory('cplex_direct',solver_io="python")
        results=opt.solve(self.control,tee=True);
    
    def update_actions():
        pass
    
    
    