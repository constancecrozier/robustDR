import csv, copy
import numpy as np
from pyomo.opt import SolverFactory, SolverManagerFactory
from controllers import MPC_model
import datetime
#import matplotlib.pyplot as plt


# This wil create an object of a smart device associated at a building

class DistNetwork:
    def __init__(self,n_bus,lmps,S,sub,t_step_min=5,n_t=12*24,
                 length=10):
        if len(lmps) < n_t:
            raise Exception('Insufficient LMPs')
        if len(sub) < n_bus:
            raise Exception('Insufficient Substation limits')
        self.n_bus = n_bus
        self.lmps = lmps
        self.t_step_min = t_step_min
        self.t_step = round(t_step_min/60,4)
        self.S = S
        self.n_t = n_t
        self.nodes = {}
        self.devices = {}
        self.len = length # the number of horizons to store data for
        for i in range(n_bus):
            self.nodes[i] = Node(i,sub[i],self.t_step,self.n_t,self.len)
        self.loaded_bdgs = {}
    
    def add_household(self,node,filepath,startdate):
        if filepath in self.loaded_bdgs:
            self.nodes[node].d += self.loaded_bdgs[filepath]
            return None
        d = []
        n_days = int((self.len*self.n_t)/(12*24/self.t_step_min))+1
        end = startdate + datetime.timedelta(n_days)
        with open(filepath,'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                date = datetime.datetime(int(row[0][:4]),
                                         int(row[0][5:7]),
                                         int(row[0][8:10]))
                if date < startdate:
                    continue
                if date > end:
                    continue
                d += [float(row[1])]*int(15/self.t_step_min)
            
            d = np.array(d[:self.len*self.n_t])
            self.loaded_bdgs[filepath] = d
            self.nodes[node].d += self.loaded_bdgs[filepath]
            return None
        
    def step_nodes(self):
        for i in self.nodes:
            self.nodes[i].d = self.nodes[i].d[1:]
    
    def add_EV(self,node,name,activity,start):
        self.devices[node,name] = EVCharger((node,name), activity, 
                                            start, self.t_step,
                                            self.t_step_min, self.n_t,
                                            copy.deepcopy(self.lmps[:self.n_t]))
            

class Controller:
    def __init__(self,network):
        self.t_step = network.t_step
        self.n_t = network.n_t
        
        
class Node:
    def __init__(self,name,P,t_step,n_t,length):
        self.id = name
        self.P = P # kW limit
        self.t_step = t_step
        self.n_t = n_t
        self.xi = [0.]*n_t
        self.d = np.array([0.]*(n_t*length))
        
class Device:
    def __init__(self,name,activity,start,t_step,t_step_min,n_t,p,E_est,
                 prices,choice='econ',thres=0):
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
        self.t_step_min = t_step_min
        self.n_t = n_t
        self.E_est = E_est# Estimated energy requirement over time_horizon
        self.E = 0#actual energy requirement (unknown by controller)
        self.p = p# Power of device
        self.prices = prices
        self.charged = 0
        self.time_passed = 0
        
        # initialisation
        self.deadline = n_t #
        self.c_thres = np.inf# the threshold price at which the vehicle will turn on
        self.x = 0#is the device drawing power
        self.econ = False
        self.load_activity_log(activity,start)
        
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
        self.econ = True
        
    def consumer_choice_now(self):
        # The consumer chooses the now setting
        self.c_thres = np.inf
        
    def consumer_choice_thres(self,c_thres):
        # The consumer chooses the price threshold setting
        self.c_thres = c_thres
        
    def is_charge(self):
        if self.econ is False:
            if self.node.prices[0] <= self.c_thres:
                self.x = 1
            else:
                self.x = 0
        else:
            mpc = MPC_model(self)
            opt = SolverFactory('cplex_direct',solver_io="python")
            results=opt.solve(mpc,tee=False)
            self.x = mpc.x[0].value
            #self.c_thres = mpc.c[device[0],device[1]]
        
    def load_activity_log(self,path='',start=datetime.datetime(2018,1,1)):
        self.events = []
        
        av_l = []
        av_en = []
        n = 0
        with open(path,'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                # start, end, energy
                s = datetime.datetime(int(row[0][:4]),int(row[0][5:7]),
                                      int(row[0][8:10]),int(row[0][11:13]),
                                      int(row[0][14:16]))
                e = datetime.datetime(int(row[1][:4]),int(row[1][5:7]),
                                      int(row[1][8:10]),int(row[1][11:13]),
                                      int(row[1][14:16]))
                if s < start:
                    continue
                try:
                    en = min(float(row[2]),66.7)
                except:
                    continue
                s = s-start
                e = e-start
                s = int((s.seconds+s.days*86400)/(60*self.t_step_min))
                e = int((e.seconds+e.days*86400)/(60*self.t_step_min))
                av_l += [e-s]
                av_en += [en]
                n += 1
                max_en = (e-s-1)*self.p*self.t_step
                en = min(max_en,en)
                #print(en,max_en)
                self.events.append([s,e,en])
        av_en = sorted(av_en)
        av_l = sorted(av_l)
        self.deadline_est = av_l[int(n*0.7)]
        self.E_est = av_en[int(n/2)]
        self.events.append([np.inf,np.inf,1.])
        if self.events[0][0] == 0:
            self.E = self.events[0][2]
            self.deadline = self.events[0][1]
            self.active = True
        
    def step(self,timestep):
        # update times of events
        for i in range(len(self.events)):
            self.events[i][0] -= 1
            self.events[i][1] -= 1
        
        # if charging update current charge
        self.E -= self.p*self.x*self.t_step
        self.charged += self.p*self.x*self.t_step
        
        if self.active is True:
            self.deadline -= 1
            self.time_passed += 1
        
        # if fully charged, plug out and remove event from log
        if self.E <= 0 and self.active is True:
            self.E = 0
            self.x = 0
            self.charged = 0
            self.time_passed = 0
            self.active = False
            print(str(timestep)+': Disconnecting device '+str(self.id)+' with no unmet demand')
            self.events.pop(0)
        
        # if device has run out of time without fulfilling needs
        if self.deadline <= 0 and self.active is True:
            print(str(timestep)+': Disconnecting device '+str(self.id)+' with '+str(self.E)+'kWh unmet demand')
            self.E = 0
            self.x = 0
            self.charged = 0
            self.time_passed = 0
            self.active = False
            self.events.pop(0)
            
            
        # if plugging in, activate
        if self.events[0][0] == 0:
            self.active = True
            #self.deadline_est = 12*8
            self.E = self.events[0][2]
            self.deadline = copy.deepcopy(self.events[0][1])
            print(str(timestep)+': Activating device '+str(self.id)+' with '
                  +str(self.E)+'kWh of demand')
                                         

class EVCharger(Device):
    def __init__(self, name, activity, start, t_step, t_step_min, n_t, prices):
        super().__init__(name, activity, start, t_step, t_step_min,  n_t, 7.0, 10.0, prices)
                                         
        
# In the pyomo model we will have variables for all smart devices, but we will fix them to off when not plugged in

# other option: directly CPLEX




#model.OBJ = pyo.Objective(expr = 2*model.x[1] + 3*model.x[2])
#model.Constraint1 = pyo.Constraint(expr = 3*model.x[1] + 4*model.x[2] >= 1)


        
        