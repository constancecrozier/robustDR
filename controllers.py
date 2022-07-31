#import csv
#import numpy as np
#import matplotlib.pyplot as plt
from pyomo.environ import (ConcreteModel, Objective, Constraint, 
                           Param, Var, Set, Boolean, minimize)

def price_model(nw):
    '''
    This functions creates a MILP optimization model for the bus pricing
    '''
    M = 1e6 # for big M formulation
    
    
    # Need to include information about current price and charging
    # That means we need to save the last price for each device
    # The energy constraint needs to consider only the time in the future
    # I should seperate LMPS and LMP forecasts, but maybe that can wait
    # I also want to update the E_est to be the previous charging demand
    
    # NEED TO RESOLVE: ISSUE WITH THE "CURRENT TIMESTEP"
    # Basically I think that I need to change the order I call functions in
    # We want to run the next prices after the actions have been updated, then step the prices, then run MPC>
    
    # parameters
    def demand(model,i,t):
        return nw.nodes[i].d[t]
    def lmps(model,t):
        return nw.lmps[t]
    def device_rating(model,i,j):
        return nw.devices[i,j].p
    def device_consumption(model,i,j):
        return max(model.p[i,j]*nw.t_step,nw.devices[i,j].E_est-nw.devices[i,j].charged)
    def sub1_lim(model):
        return nw.S
    def sub2_lim(model,i):
        return nw.nodes[i].P
    def slack(model):
        return 1e6
    
    
    # info: both xi0 and x0 are zero, says problem is infeasible
    
    # constraints
    def xi_sum(model,i):
        return sum([model.xi[i,t] for t in model.time_set]) == 0
    def c_def(model,i,j,t):
        return (model.lmp[t]+model.xi[i,t] <= model.c[i,j] + M*(1-model.x[i,j,t]))
    def c_def2(model,i,j,t):
        return (model.c[i,j] <= model.lmp[t]+model.xi[i,t] + M*model.x[i,j,t])
    def x0(model,i,j):
        return model.x[i,j,0] == nw.devices[i,j].x
    def xi0(model,i):
        return model.xi[i,0] == nw.devices[i,0].prices[0]-nw.lmps[0]
    def en_req(model,i,j):
        return sum([model.x[i,j,t] for t in model.time_set])*model.p[i,j]*nw.t_step >= model.E[i,j] #- model.sigmaj[i,j]
    def sub1(model,t):
        return (sum(model.x[i,j,t]*model.p[i,j] for (i,j) in model.device_set) <=
                model.S - sum(model.d[i,t] for i in model.bus_set) + model.sigma[t])
    def sub2(model,i,t):
        return (sum([model.x[i2,j,t]*model.p[i2,j] for (i2,j) in model.device_set 
                     if i2 == i])<= model.P[i] - model.d[i,t])
    
    # objective
    def cost(model):
        c = sum([model.c[i,j] for (i,j) in model.device_set])
        for (i,j) in model.device_set:
            for t in model.time_set:
                c += model.x[i,j,t]*model.p[i,j]*model.lmp[t]
        for t in model.time_set:
            c += model.sigma[t]*model.kappa
        #for (i,j) in model.device_set:
        #    c += model.sigmaj[i,j]*model.kappa
        return c


    model = ConcreteModel()
    
    # Sets
    model.bus_set=Set(initialize=list(nw.nodes.keys()))
    model.device_set=Set(initialize=[d for d in list(nw.devices.keys()) if nw.devices[d].active==True])
    model.time_set=Set(initialize=list(range(nw.n_t)))

    # Parameters
    model.d = Param(model.bus_set,model.time_set,rule=demand)
    model.lmp = Param(model.time_set,rule=lmps)
    model.p = Param(model.device_set,rule=device_rating)
    model.P = Param(model.bus_set,rule=sub2_lim)
    model.S = Param(rule=sub1_lim)
    model.E = Param(model.device_set,rule=device_consumption)
    model.kappa = Param(rule=slack)

    # Variables
    model.x = Var(model.device_set, model.time_set, within=Boolean)
    model.xi = Var(model.bus_set, model.time_set)
    model.c = Var(model.device_set)
    model.sigma = Var(model.time_set, bounds=(0,1000))
    #model.sigmaj = Var(model.device_set, bounds=(0,100))
    
    for (i,j) in model.device_set:
        if ((nw.devices[i,j].deadline_est-nw.devices[i,j].time_passed)*model.p[i,j]*nw.t_step
            > model.E[i,j] and nw.devices[i,j].deadline_est-nw.devices[i,j].time_passed > 0):
            for t in range(nw.devices[i,j].deadline_est-nw.devices[i,j].time_passed,nw.n_t):
                model.x[i,j,t].fix(0)

    # Constraints    
    model.xi_sum = Constraint(model.bus_set,rule=xi_sum)
    model.c_def = Constraint(model.device_set,model.time_set,rule=c_def)
    model.c_def2 = Constraint(model.device_set,model.time_set,rule=c_def2)
    model.en_req = Constraint(model.device_set,rule=en_req)
    model.xi0 = Constraint(model.bus_set,rule=xi0)
    model.x0 = Constraint(model.device_set,rule=x0)
    
    if len(model.device_set) > 0:
        model.sub1 = Constraint(model.time_set,rule=sub1)
        #model.sub2 = Constraint(model.bus_set,model.time_set,rule=sub2)
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model

def MPC_model(device):
    '''
    This function creates an MILP optimization problem for individual devices
    to optimize their consumption
    '''
    
    def prices(model,t):
        return device.prices[t]
    
    def en_req(model):
        return sum([model.x[t]*device.p*device.t_step for t in model.time_set]) + model.sigma >= device.E
    
    def cost(model):
        return sum([model.x[t]*model.c[t] for t in model.time_set]) + model.sigma*1e8
    
    model = ConcreteModel()
    
    # Sets
    model.time_set=Set(initialize=list(range(min(device.deadline,device.n_t))))
    
    # Parameters
    model.c = Param(model.time_set, rule=prices)
    
    # Variables
    model.x = Var(model.time_set, within=Boolean)
    model.sigma = Var(bounds=(0,10))
    
    # Constraints
    model.en = Constraint(rule=en_req)
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model

def direct_control_model(nw):
    '''
    This functions creates a MILP optimization model for the bus pricing
    '''
    M = 1e6 # for big M formulation
    
    # parameters
    def demand(model,i,t):
        return nw.nodes[i].d[t]
    def lmps(model,t):
        return nw.lmps[t]
    def device_rating(model,i,j):
        return nw.devices[i,j].p
    def device_consumption(model,i,j):
        return nw.devices[i,j].E
    def sub1_lim(model):
        return nw.S
    def sub2_lim(model,i):
        return nw.nodes[i].P
    def slack(model):
        return 1e8
    
    # constraints
    def en_req(model,i,j):
        return sum([model.x[i,j,t] for t in model.time_set])*model.p[i,j]*nw.t_step >= model.E[i,j]
    def sub1(model,t):
        return (sum(model.x[i,j,t]*model.p[i,j] for (i,j) in model.device_set) <=
                model.S - sum(model.d[i,t] for i in model.bus_set) + model.sigma[t])
    #def sub2(model,i,t):
    #    return (sum([model.x[i2,j,t]*model.p[i2,j] for (i2,j) in model.device_set if i2 == i])<= model.P[i] - model.d[i,t])
    
    # objective
    def cost(model):
        c = 0.
        for (i,j) in model.device_set:
            for t in model.time_set:
                c += model.x[i,j,t]*model.p[i,j]*model.lmp[t]
        for t in model.time_set:
            c += model.sigma[t]*model.kappa
        return c


    model = ConcreteModel()
    
    # Sets
    model.bus_set=Set(initialize=list(nw.nodes.keys()))
    model.device_set=Set(initialize=[d for d in list(nw.devices.keys()) if nw.devices[d].active==True])
    model.time_set=Set(initialize=list(range(nw.n_t)))

    # Parameters
    model.d = Param(model.bus_set,model.time_set,rule=demand)
    model.lmp = Param(model.time_set,rule=lmps)
    model.p = Param(model.device_set,rule=device_rating)
    #model.P = Param(model.bus_set,rule=sub2_lim)
    model.S = Param(rule=sub1_lim)
    model.E = Param(model.device_set,rule=device_consumption)
    model.xi = Param(model.bus_set, model.time_set,rule=0)
    model.kappa = Param(rule=slack)

    # Variables
    model.x = Var(model.device_set, model.time_set, within=Boolean)
    model.sigma = Var(model.time_set, bounds=(0,1000))


    # Constraints 
    model.en_req = Constraint(model.device_set,rule=en_req)
    
    if len(model.device_set) > 0:
        model.sub1 = Constraint(model.time_set,rule=sub1)
        #model.sub2 = Constraint(model.bus_set,model.time_set,rule=sub2)
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model
