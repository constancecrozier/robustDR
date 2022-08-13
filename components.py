import csv, copy
import numpy as np
from pyomo.opt import SolverFactory, SolverManagerFactory
from controllers import MPC_model
import datetime
#import matplotlib.pyplot as plt


# This wil create an object of a smart device associated at a building

class DistNetwork:
    def __init__(self,n_bus,lmps,lmp_est,S,sub=None,t_step_min=5,n_t=12*24,
                 length=10):
        if len(lmps) < n_t:
            raise Exception('Insufficient LMPs')
        if len(lmp_est) < n_t:
            raise Exception('Insufficient LMP estimates')
            
        if sub is None:
            sub = [np.inf]*n_bus
        self.n_bus = n_bus
        self.lmp_true = lmps
        self.lmp_est = lmp_est
        self.lmps = lmps
        self.lmps = (copy.deepcopy(self.lmp_true[:int(60/t_step_min)])
                     +copy.deepcopy(self.lmp_est[int(60/t_step_min):]))
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
    
    def add_EV(self,node,name,activity,start,choice='econ'):
        self.devices[node,name] = EVCharger((node,name), activity, 
                                            start, self.t_step,
                                            self.t_step_min, self.n_t,
                                            copy.deepcopy(self.lmps[:self.n_t]),
                                            choice=choice)
        
    def add_HVAC(self,node,name,temperature,start,R,C,T0,T_min,T_max):
        self.devices[node,name+'a'] = Heating((node,name),self.t_step,
                                              self.t_step_min, self.n_t,
                                              copy.deepcopy(self.lmps[:self.n_t]),
                                              temperature,start,R,C,T0,T_min)
        self.devices[node,name+'b'] = Cooling((node,name),self.t_step,
                                              self.t_step_min, self.n_t,
                                              copy.deepcopy(self.lmps[:self.n_t]),
                                              temperature,start,R,C,T0,T_max)
    

            
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
    def __init__(self,name,t_step,t_step_min,n_t,prices,p,eta,
                 typ='deadline',interruptile=True,
                 activity=None,temp=None,startdate=None,
                 T_min=None, T_max=None, R=None, C=None, T0=None,
                 choice='econ',c_thres=np.inf):
        '''
        name (str): identifier of individual device
        t_step (float): size of timestep (hours)
        t_step_min (float): size of timestep (mins)
        n_t (int): number of timesteps in the simulation
        prices (Array): list of forecast DR prices
        p (float): rating of device (kW)
        eta (float): device efficiency (%)
        
        typ (str): type of device either 'deadline' or 'thermal'
        interruptile (boo): whether the demand can be interrupted
        
        activity (str): filepath to activity log
        temp (str): filepath to temperature log
        startdate (datetime): beginning of simulation
        
        T_min (float): lower building temperature bound (degrees C)
        T_max (float): upper building temperature bound (degrees C)
        r (float):
        c (float):
        T0 (float): intial building temperature (degrees C)
        
        choice (str): for deadline devices, econ, c_thres, or now
        c_thres (float): the upper charging limit ($/kWh)
        '''
        
        # Some checks
        if choice not in ['econ','c_thres','now']:
            raise Exception('choice not recognized')
        if typ not in ['deadline', 'thermal']:
            raise Exception('type not recognized')
        if typ == 'thermal' and interruptile == False:
            raise Exception('thermal devices must be interruptible')
        if typ == 'thermal' and (t_min == None and t_max == None):
            raise Exception('thermal devices require a temperature bound')
        if typ == 'thermal' and (R == None or C == None):
            raise Exception('thermal devices require r and c values')
            
        # Simulation properties
        self.t_step = t_step
        self.t_step_min = t_step_min
        self.n_t = n_t 
        self.prices = prices
        
        # Device properties
        self.id = name
        self.node = name[0]
        self.p = p
        self.eta = eta
        self.type = typ
        self.interruptile = interruptile
        self.R = R
        self.C = C
        self.T_min = T_min
        self.T_max = T_max
        
        # Initialization
        self.active = False # is device connected
        self.E = 0 # actual energy required (kWh)
        self.x = 0 # is device drawing power
        self.T = T0
        self.deadline = n_t 
        self.charged = 0
        self.time_passed = 0 
        
        # Consumer preferences
        self.c_thres = c_thres
        if choice == 'econ':
            self.econ = True
        else:
            self.econ = False
        if typ == 'deadline':
            self.load_activity_log(activity,start)
        else:
            self.load_temperature_log(activity,start)
            
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
            if self.prices[0] <= self.c_thres:
                self.x = 1
            else:
                self.x = 0
        else:
            mpc = MPC_model(self,self.type,self.interruptible)
            opt = SolverFactory('cplex_direct',solver_io="python")
            results=opt.solve(mpc,tee=False)
            self.x = mpc.x[0].value
        
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
    
    def load_temperature_log(self,path='',start=datetime.datetime(2018,1,1)):
        return None
        
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
    def __init__(self, name, activity, start, t_step, t_step_min, n_t, prices, 
                 choice='econ'):
        super().__init__(name, t_step, t_step_min, n_t, prices, 7.0, 0.9,
                         typ='deadline',interruptile=True,
                         activity=activity,startdate=start,choice=choice)
                                         
class Heating(Device):
    def __init__(self, name, t_step, t_step_min, n_t, prices, temp,
                 start, R, C, T0, T_min):
        super().__init__(name, t_step, t_step_min, n_t, prices, 4.0, 0.7,
                         typ='thermal', temp=temp, startdate=start,
                         T_min=T_min, R=R, C=C, T0=T0)


#class AC(Device):


        