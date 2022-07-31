import csv, datetime, copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pyomo.opt import SolverFactory, SolverManagerFactory
from controllers import price_model, MPC_model, direct_control_model

# to do, get data for LMPs
# check lmps are included in the write data file
# build iteration

def get_casio_lmps(startdate,enddate,resolution=60):
    data_60 = np.zeros((24*(enddate-startdate).days,))
    with open('data/CAISO_LMP.csv','r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            day = datetime.datetime(int(row[0][:4]),int(row[0][5:7]),int(row[0][8:10]))
            if day > enddate or day < startdate:
                continue
            d = (day-startdate).days
            h = int(row[1])-1
            data_60[d*24+h] = float(row[2])
    
    if resolution == 60:
        return data_60
    
    data = []
    for t in range(len(data_60)):
        for n in range(int(60/resolution)):
            data.append(data_60[t])
    return data

def get_casio_en(startdate,enddate,resolution=60):
    data_60 = np.zeros((24*(enddate-startdate).days,))
    with open('data/CAISO_EN.csv','r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            day = datetime.datetime(int(row[0][:4]),int(row[0][5:7]),int(row[0][8:10]))
            if day > enddate or day < startdate:
                continue
            d = (day-startdate).days
            h = int(row[1])
            data_60[d*24+h] = float(row[2])
    
    if resolution == 60:
        return data_60
    
    data = []
    for t in range(len(data_60)):
        for n in range(int(60/resolution)):
            data.append(data_60[t])
    return data


def get_comed(startdate,enddate):
    # NOTE: DATA IS AT 5MIN RESOLUTION
    # TO DO: Add functionality to change time resolution
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
            self.timestep(opt,t)
            self.write_results(t)
            
    def run_benchmark(self,n_t,results_filepath='results/sim'):
        self.path = results_filepath
        self.create_results_file()
        # we need a new step to feed the TRUE energy values to 
        for t in range(n_t):
            self.timestep_benchmark()
            self.write_results(t)
    
    def update_device_actions(self):
        # Each device makes its charging decisions
        for device in self.nw.devices:
            if self.nw.devices[device].active == False:
                continue
            self.nw.devices[device].is_charge()
    
    def step_devices(self,t):
        for device in self.nw.devices:
            self.nw.devices[device].step(t)
        
    def timestep(self,opt,t):
        self.update_device_actions()
        self.update_prices(int(opt),t)
        self.nw.step_nodes()
        self.step_devices(t)
        self.nw.lmps = self.nw.lmps[1:]
        
    def timestep_benchmark(self):
        self.controller = direct_control_model(self.nw)
        sol = SolverFactory('cplex_direct',solver_io="python")
        results=sol.solve(self.controller,tee=False)
        # pass down prices to nodes
        for (i,j) in self.nw.devices:
            for t in self.controller.time_set:
                self.nw.devices[i,j].prices[t] = self.controller.xi[i,t]+self.nw.lmps[t]
            if self.nw.devices[i,j].active == True:
                self.nw.devices[i,j].x = self.controller.x[i,j,0].value
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
            try:
                newline += str(self.controller.xi[i,0].value)+','
            except:
                newline += 'NaN,'
        for i in list(self.nw.nodes.keys()):
            newline += str(self.nw.nodes[i].d[0])+','
        for j in list(self.nw.devices.keys()):
            newline += str(self.nw.devices[j].x*self.nw.devices[j].p)+','
        newline = newline[:-1]+'\n'
        f = open(self.path+'.txt','a')
        f.write(newline)
        f.close()
    
    def update_prices(self,opt,t):
        self.controller = price_model(self.nw)
        if opt == 1:
            sol = SolverFactory('cplex_direct',solver_io="python")
            results=sol.solve(self.controller,tee=False)
        if self.controller.sigma[0].value > 1e-3:
            print(str(t)+': constraint violation '
                  +str(self.controller.sigma[0].value))
            
        # pass down prices to nodes
        for (i,j) in self.nw.devices:
            for t in self.controller.time_set:
                self.nw.devices[i,j].prices[t] = self.nw.lmps[t+1]
                if (opt == 1 and t < len(self.controller.time_set)-1 
                    and len(self.controller.device_set) > 0):
                    self.nw.devices[i,j].prices[t] += self.controller.xi[i,t+1].value
                
class Sim_Plot:
    def __init__(self,nw,xstart=0,xend=None):
        plt.rcParams["font.family"] = 'serif'
        plt.rcParams["font.size"] = '10'
        
        self.xstart = xstart
        self.xend = xend
        
        self.fig = plt.figure()
        gs = GridSpec(2, 2, figure=self.fig)
        self.ax1 = self.fig.add_subplot(gs[0, :])
        self.ax2 = self.fig.add_subplot(gs[1,0])
        self.ax3 = self.fig.add_subplot(gs[1,1])

        self.lmps = nw.lmps

        # plotting demand
        d = None
        for i in nw.nodes:
            if d is None:
                d = copy.deepcopy(nw.nodes[i].d)
            else:
                d += nw.nodes[i].d
        self.d = d
        self.p = {}
        self.xi = {}
        self.ax1.plot(d,label='Base',c='k',ls=':')

        if xend is None:
            xend = nw.n_t*nw.len

        # change x range
        self.ax1.set_xlim(xstart,xend)

        # set titles
        self.ax1.set_title('Power Demand',y=0.8)
        self.ax2.set_title('Cost',y=0.8)
        self.ax3.set_title('Energy',y=0.8)

        # change x scale to hours
        sf = int(60/nw.t_step_min)
        self.t_step = nw.t_step
        #self.ax1.set_xticks(np.arange(0,int(xend/sf)*sf,sf),
        #                    labels=np.arange(0,int(xend/sf),1))
        #self.ax2.set_xticks(np.arange(0,int(xend/sf)*sf,sf),
        #                    labels=np.arange(0,int(xend/sf),1))

        # plot transformer limit
        self.ax1.plot([xstart,xend],[nw.S,nw.S],c='r',ls='--')
    
    def plot_prices(self,name):
        plt.figure()
        plt.subplot(1,2,1)
        plt.plot(self.lmps[self.xstart:self.xend])
        plt.subplot(1,2,2)
        for i in self.xi[name]:
            plt.plot(self.xi[name][i][self.xstart:self.xend])
        plt.show()
        
    def add_simulation(self,sim,name):
        with open(sim+'.txt','r') as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader)
            i = 2
            n_i = 0
            n_j = 0
            xi = {}
            p = {}
            while header[i][0] == 'x':
                n_i += 1
                i += 1
            n_j = len(header)-i-n_i
            for i in range(n_i):
                xi[i] = []
            for j in range(n_j):
                p[j] = []
            for row in reader:
                for i in range(n_i):
                    try:
                        xi[i].append(float(row[2+i]))
                    except:
                        xi[i].append(None)
                for j in range(n_j):
                    p[j].append(float(row[2+n_i+n_i+j]))
        self.p[name] = p
        self.xi[name] = xi
        
    def plot_simulation(self,sim,name,n):
        self.add_simulation(sim,name)
        
        p_total = list(copy.deepcopy(self.d))
        cost = [0.]*len(p_total)
        for i in self.p[name]:
            for t in range(len(self.p[name][i])):
                p_total[t] += self.p[name][i][t]
                cost[t] += self.p[name][i][t]*self.lmps[t]*self.t_step
        # now make ZOH version
        _t = [0]
        p = [p_total[0]]
        for t in range(len(p_total)-1):
            p += [p_total[t],p_total[t+1]]
            _t += [t+1,t+1]
        cost = cost[self.xstart:self.xend]
        p_total = p_total[self.xstart:self.xend]
        
        energy = (sum(p_total)-sum(self.d[self.xstart:self.xend]))*self.t_step
        self.ax1.plot(_t,p,label=name)
        self.ax2.bar([n],[sum(cost)/energy],label=name)
        self.ax3.bar([n],[energy],label=name)
    
    def save_plot(self):
        self.fig.savefig('test.png')
        plt.show()
    
    
                
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
                
    def plot_total_load(self,downsample=1,lim=None):
        d_only = np.zeros((self.n_t,))
        dev1 = np.zeros((self.n_t,))
        dev2 = np.zeros((self.n_t,))
        
        for t in range(self.n_t):
            for i in range(self.n_i):
                d_only[t] += self.d[i][t]
            for j in range(self.n_j):
                dev1[t] += self.p1[j][t]
                dev2[t] += self.p2[j][t]
        
        dev1 += d_only
        dev2 += d_only
        
        if downsample > 1:
            _d_only = np.zeros((int(self.n_t/downsample),))
            _dev1 = np.zeros((int(self.n_t/downsample),))
            _dev2 = np.zeros((int(self.n_t/downsample),))
            
            for t in range(int(self.n_t/downsample)):
                for n in range(downsample):
                    _d_only[t] += d_only[t*downsample+n]/downsample
                    _dev1[t] += dev1[t*downsample+n]/downsample
                    _dev2[t] += dev2[t*downsample+n]/downsample
            dev1 = _dev1
            dev2 = _dev2
            d_only = _d_only
            
        d = [d_only[0]]
        p1 = [dev1[0]]
        p2 = [dev2[0]]
        x = [0]
          
        plt.figure()
        plt.plot(x,d,c='k',ls=':')
        plt.plot(x,p1)
        plt.plot(x,p2)
        plt.plot([0,len(d_only)],[lim,lim],c='r',ls='--')
        plt.show()
        

    
    