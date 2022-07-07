import csv, datetime
import numpy as np
import matplotlib.pyplot as plt
from pyomo.opt import SolverFactory, SolverManagerFactory
from controllers import price_model, MPC_model

# to do, get data for LMPs
# check lmps are included in the write data file
# build iteration

def get_comed(startdate,enddate):
    with open('data/comed.csv','r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        data = []
        for row in reader:
            # NOTE: you need to manually change the offset of your local machine compared to PST
            d = datetime.datetime.fromtimestamp(int(row[0])/1000) - datetime.timedelta(hours=1)
            if d < startdate:
                continue
            if d > enddate:
                continue
            data.append([d,float(row[1])])
        data = sorted(data)
        lmp = [data[i][1] for i in range(len(data))]
    return lmp
            

class Simulation:
    def __init__(self,network):
        self.nw = network
        
    def run_simulation(self,n_t,results_filepath='results/sim',opt=True):
        self.path = results_filepath
        self.create_results_file()
        for t in range(n_t):
            self.timestep(opt)
            self.write_results(t)
    
    def update_device_actions(self):
        # Each device makes its charging decisions
        for device in self.nw.devices:
            if self.nw.devices[device].active == False:
                continue
            self.nw.devices[device].is_charge()
    
    def step_devices(self):
        for device in self.nw.devices:
            self.nw.devices[device].step()
        
    def timestep(self,opt):
        if opt is True:
            self.update_prices()
        self.update_device_actions()
        self.nw.step_nodes()
        self.step_devices()
        self.nw.lmps = self.nw.lmps[1:]
    
    def create_results_file(self):
        f = open(self.path+'.txt','w')
        headers = 't,lambda,'
        for i in list(self.nw.nodes.keys()):
            headers += 'xi'+str(i)+','
        for i in list(self.nw.nodes.keys()):
            headers += 'demand'+str(i)+','
        for j in list(self.nw.devices.keys()):
            headers += 'device'+str(j[0])+str('_')+str(j[1])+','
        headers = headers[:-1]+'\n'
        f.write(headers)
        f.close()
    
    def write_results(self,timestep):
        newline = str(timestep)+','+str(self.nw.lmps[0])+','
        for i in list(self.nw.nodes.keys()):
            newline += str(self.controller.xi[i,0].value)+','
        for i in list(self.nw.nodes.keys()):
            newline += str(self.nw.nodes[i].d[0])+','
        for j in list(self.nw.devices.keys()):
            newline += str(self.nw.devices[j].x*self.nw.devices[j].p)+','
        newline = newline[:-1]+'\n'
        f = open(self.path+'.txt','a')
        f.write(newline)
        f.close()
    
    def update_prices(self):
        self.controller = price_model(self.nw)
        opt = SolverFactory('cplex_direct',solver_io="python")
        results=opt.solve(self.controller,tee=False)
        # pass down prices to nodes
        for (i,j) in self.nw.devices:
            for t in self.controller.time_set:
                self.nw.devices[i,j].prices[t] = self.controller.xi[i,t].value+self.nw.lmps[t]
                
class Sim_Comparison:
    
    def __init__(self,sim1,sim2):
        self.sim1 = sim1
        self.sim2 = sim2
        
        self.xi = {}
        self.p1 = {}
        self.p2 = {}
        self.d = {}
        self.lmp = []
        with open(sim1+'.txt','r') as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader)
            i = 2
            n_i = 0
            n_j = 0
            while header[i][0] == 'x':
                n_i += 1
                i += 1
            n_j = len(header)-i-n_i
            self.n_i = n_i
            self.n_j = n_j
            for i in range(n_i):
                self.d[i] = []
                self.xi[i] = []
            for j in range(n_j):
                self.p1[j] = []
                self.p2[j] = []
            for row in reader:
                self.lmp.append(float(row[1]))
                for i in range(n_i):
                    self.xi[i].append(float(row[2+i]))
                    self.d[i].append(float(row[2+i+n_i]))
                for j in range(n_j):
                    self.p1[j].append(float(row[2+n_i+n_i+j]))
            self.n_t = len(self.d[0])
                    
        with open(sim2+'.txt','r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                for j in range(n_j):
                    self.p2[j].append(float(row[2+n_i+n_i+j]))
                
    def plot_total_load(self):
        d_only = np.zeros((self.n_t,))
        dev1 = np.zeros((self.n_t,))
        dev2 = np.zeros((self.n_t,))
        
        for t in range(self.n_t):
            for i in range(self.n_i):
                d_only[t] += self.d[i][t]
            for j in range(self.n_j):
                dev1[t] += self.p1[j][t]
                dev2[t] += self.p2[j][t]
        
        plt.figure()
        plt.plot(d_only,c='k',ls=':')
        plt.plot(dev1+d_only)
        plt.plot(dev2+d_only)
        plt.show()
        

    
    