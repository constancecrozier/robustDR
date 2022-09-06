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
        self.buildings = {}
    
    def add_building(self,node_id,building_id,filepath,startdate,T_out,GHI,
                     heating=True,cooling=True,k=1.3,iR=60,iC=6.667e-8,
                     T0=20):
        self.buildings[node_id,building_id] = Building(node_id,building_id,self.t_step,k,iR,iC,T0,T_out,GHI)
        
        if filepath in self.loaded_bdgs:
            self.nodes[node_id].d += self.loaded_bdgs[filepath]
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
                p = float(row[1])
                if heating == False:
                    p -= float(row[2])
                if cooling == False:
                    p -= float(row[3])
                d += [p]*int(15/self.t_step_min)
                
            
            d = np.array(d[:self.len*self.n_t])
            self.loaded_bdgs[filepath] = d
            self.nodes[node_id].d += self.loaded_bdgs[filepath]
            return None
        
    def step_nodes(self):
        for i in self.nodes:
            self.nodes[i].d = self.nodes[i].d[1:]
        for (i,b) in self.buildings:
            self.buildings[i,b].step_building()
    
    def add_EV(self,node_id,building_id,device_id,activity,start,choice='econ'):
        self.devices[node_id,building_id,device_id] = EVCharger(node_id, building_id, device_id,
                                                                activity, start, self.t_step,
                                                                self.t_step_min, self.n_t,
                                                                copy.deepcopy(self.lmps[:self.n_t]),
                                                                choice=choice)
        
        
    def add_HVAC(self,node_id,building_id,device_id,T_min,T_max,T0=None):
        if T0 is not None:
            self.buildings[node_id,building_id].T0 = T0
        
        
        self.devices[node_id,building_id,device_id+'a'] = Heating(node_id, building_id, device_id+'a',
                                                                  self.t_step,self.t_step_min, self.n_t,
                                                                  copy.deepcopy(self.lmps[:self.n_t]),
                                                                  T_min)#'''
        
        self.devices[node_id,building_id,device_id+'b'] = Cooling(node_id, building_id, device_id+'b',
                                                                  self.t_step,self.t_step_min, self.n_t,
                                                                  copy.deepcopy(self.lmps[:self.n_t]),
                                                                  T_max)
        
        
        
class Node:
    def __init__(self,name,P,t_step,n_t,length):
        self.id = name
        self.P = P # kW limit
        self.t_step = t_step
        self.n_t = n_t
        self.xi = [0.]*n_t
        self.d = np.array([0.]*(n_t*length))
        self.buildings = {}
        
class Building:
    def __init__(self,node_id,building_id,t_step,k,iR,iC,T0,T_out,GHI):
        self.node_id = node_id
        self.building_id = building_id
        self.t_step = t_step
        self.k = k
        self.iR = iR
        self.iC = iC
        self.T = T0
        self.T_out = T_out
        self.GHI = GHI
        
    def step_building(self):
        # THIS UPDATES THE BUILDING TEMP CONSIDERING NO DEVICES
        self.T += ((self.T_out[0]-self.T)*self.iR*self.iC
                    +self.k*self.GHI[0]*self.iC)*3600*self.t_step
        self.T_out = self.T_out[1:]
        self.GHI = self.GHI[1:]
        
class Device:
    def __init__(self, node_id, building_id, device_id,
                 t_step, t_step_min, n_t, prices, p, eta,
                 typ='deadline', interruptible=True, 
                 activity=None, startdate=None, T_min=0., T_max=100.,
                 choice='econ', c_thres=np.inf):
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
        if typ == 'thermal' and interruptible == False:
            raise Exception('thermal devices must be interruptible')
        if typ == 'thermal' and (T_min == 0. and T_max == 100.):
            raise Exception('thermal devices require a temperature bound')
            
        # Simulation properties
        self.t_step = t_step
        self.t_step_min = t_step_min
        self.n_t = n_t 
        self.prices = prices
        
        # Device properties
        self.device_id = device_id
        self.node_id = node_id
        self.building_id = building_id
        self.p = p
        self.eta = eta
        self.type = typ
        self.interruptible = interruptible
        self.T_min = T_min
        self.T_max = T_max
        
        # Initialization
        self.active = True # is device connected
        self.E = 0 # actual energy required (kWh)
        self.x = 0 # is device drawing power
        self.deadline = np.inf 
        self.charged = 0
        self.time_passed = 0 
        
        # Consumer preferences
        self.c_thres = c_thres
        if choice == 'econ':
            self.econ = True
        else:
            self.econ = False
        if typ == 'deadline':
            self.load_activity_log(activity,startdate)
        #else:
        #    self.load_temperature_log(activity,start)
            
    def consumer_choice_econ(self):
        # The consumer chooses the economy setting
        self.econ = True
        
    def consumer_choice_now(self):
        # The consumer chooses the now setting
        self.c_thres = np.inf
        
    def consumer_choice_thres(self,c_thres):
        # The consumer chooses the price threshold setting
        self.c_thres = c_thres
        
    def is_on(self,building):
        if self.econ is False:
            if self.prices[0] <= self.c_thres:
                self.x = 1
            else:
                self.x = 0
        else:
            mpc = MPC_model(self,self.type,self.interruptible,building)
            opt = SolverFactory('cplex_direct',solver_io="python")
            opt.options['timelimit'] = 20
            results=opt.solve(mpc,tee=False)
            self.x = mpc.x[0].value
        
    def load_activity_log(self,path='',start=datetime.datetime(2018,1,1)):
        self.events = []
        self.active = False
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
    
    def step(self,timestep,building):
        if self.type == 'deadline':
            self.step_deadline(timestep)
        else:
            self.step_thermal(timestep,building)
    
    def step_thermal(self,timestep,building):
        # adds device contribution to thermal
        building.T += (self.p*1000*self.x*self.eta*building.iC)*3600*self.t_step
        
        # we do not want both AC and Heating to be active at once
        # so can we find a way to deactivate unnecessary devices
        
        if self.T_min > 0 and building.T > building.T_out[0]:
            self.active = True
        elif self.T_max < 100 and building.T < building.T_out[0]:
            self.active = True
        else:
            self.active = False
            self.x = 0
            
        
        
        
    def step_deadline(self,timestep):
        # update times of events
        for i in range(len(self.events)):
            self.events[i][0] -= 1
            self.events[i][1] -= 1
        
        # if charging update current charge
        self.E -= self.p*self.x*self.eta*self.t_step
        self.charged += self.p*self.x*self.eta* self.t_step
        
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
            print(str(timestep)+': Disconnecting device '+self.device_id+' at Node '
                  +str(self.node_id))
            self.events.pop(0)
        
        # if device has run out of time without fulfilling needs
        if self.deadline <= 0 and self.active is True:
            print(str(timestep)+': Disconnecting device '+self.device_id+' at Node '
                  +str(self.node_id)+' with '+str(self.E)+'kWh unmet demand')
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
            print(str(timestep)+': Activating device '+self.device_id+' at Node '
                  +str(self.node_id))
                                      

class EVCharger(Device):
    def __init__(self, node_id, building_id, device_id, activity, start, t_step, 
                 t_step_min, n_t, prices, choice='econ'):
        super().__init__(node_id, building_id, device_id, t_step, t_step_min, 
                         n_t, prices, 7.0, 0.9, typ='deadline', interruptible=True,
                         activity=activity, startdate=start, choice=choice)
                                         
class Heating(Device):
    def __init__(self, node_id, building_id, device_id, t_step, t_step_min, n_t, 
                 prices, T_min):
        # p_heat = 4.5 kW thermal 
        # hspf = 8.2
        # hspf --> COP = 0.293
        # p = 4.5/(8.2*0.293) = 1.87 kW
        # eta = 8.2*0.293 = 2.4
        super().__init__(node_id, building_id, device_id, t_step, t_step_min, 
                         n_t, prices, 1.87, 2.4, typ='thermal', 
                         T_min=T_min)
        
class Cooling(Device):
    def __init__(self, node_id, building_id, device_id, t_step, t_step_min, n_t, 
                 prices, T_max):
        # p_cool = 7 kW thermal
        # seer = 16
        # seer --> COP = 0.293
        # p = 7/(16*0.293) = 1.5 kW
        # eta = 16*0.293 = 4.68
        super().__init__(node_id, building_id, device_id, t_step, t_step_min, 
                         n_t, prices, 1.5, -4.68, typ='thermal', 
                         T_max=T_max)


#class AC(Device):


        