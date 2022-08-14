import copy
from pyomo.environ import (ConcreteModel, Objective, Constraint, 
                           Param, Var, Set, Boolean, minimize)

def price_model(nw, requirements=True, M=1e6):
    '''
    This functions creates a MILP optimization model for the bus pricing
    
    nw (Network object): parameterized network
    
    requirements (boo): Whether or not to give correct requirements to price controller
    
    M (float): constant for big M formulation
    '''
    
    # TO DO:
    # The requirements is False method is still buggy, we need to re-adjust estiamtes when things aren't makeing sense
        
    # === PARAMETERS ===
    
    def demand(model,i,t):
        return nw.nodes[i].d[t]
    def lmps(model,t):
        return nw.lmps[t]
    def device_rating(model,i,j):
        return nw.devices[i,j].p
    def device_eff(model,i,j):
        return nw.devices[i,j].eta
    def device_consumption(model,i,j):
        if requirements is True:
            return nw.devices[i,j].E
        return max(model.p[i,j]*nw.t_step,nw.devices[i,j].E_est-nw.devices[i,j].charged,
                   model.p[i,j]*nw.t_step*nw.devices[i,j].x)
    def sub1_lim(model):
        return nw.S
    def sub2_lim(model,i):
        return nw.nodes[i].P
    def slack(model):
        return 1e6
    def deadline(model,i,j):
        # first check whether vehicle is accepting high prices
        if nw.devices[i,j] == 1 and nw.devices[i,0].prices[0] > 1e3:
            # return minimum time to charge, will fully constrain device
            return int(model.E[i,j]/(model.p[i,j]*nw.t_step))+1
        if requirements is True:
            return nw.devices[i,j].deadline
        else:
            return nw.devices[i,j].deadline_est-nw.devices[i,j].time_passed
    
    # === CONSTRAINTS ===
    
    # adders sum to zero
    def xi_sum(model,i):
        return sum([model.xi[i,t] for t in model.time_set]) == 0
    
    # big M rule for price sensitiivty of devices
    def c_def(model,i,j,t):
        if t < model.deadline[i,j]:
            return (model.lmp[t]+model.xi[i,t] <= model.c[i,j] + M*(1-model.x[i,j,t]))
        else:
            return Constraint.Skip
        
    def c_def2(model,i,j,t):
        if t < model.deadline[i,j]:
            return (model.c[i,j] <= model.lmp[t]+model.xi[i,t] + M*model.x[i,j,t])
        else:
            return Constraint.Skip
    
    # constrain observed price and charging behaviour
    def x0(model,i,j):
        return model.x[i,j,0] == nw.devices[i,j].x
    
    def xi0(model,i):
        return model.xi[i,0] == nw.devices[i,0].prices[0]-nw.lmps[0]
    
    # energy requirement of the devices
    def en_req(model,i,j):
        return sum([model.x[i,j,t] for t in model.time_set])*model.p[i,j]*nw.t_step >= model.E[i,j] 
    
    # transformer limit
    def sub1(model,t):
        return (sum(model.x[i,j,t]*model.p[i,j] for (i,j) in model.device_set) <=
                model.S - sum(model.d[i,t] for i in model.bus_set) + model.sigma[t])
    
    #def sub2(model,i,t):
    #    return (sum([model.x[i2,j,t]*model.p[i2,j] for (i2,j) in model.device_set 
    #                 if i2 == i])<= model.P[i] - model.d[i,t])
    
    # objective
    def cost(model):
        c = sum([model.c[i,j] for (i,j) in model.device_set])
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
    model.dead_dev_set=Set(initialize=[d for d in list(nw.devices.keys()) if nw.devices[d].active==True 
                                       and nw.devices[c].typ == 'deadline'])
    model.therm_dev_set=Set(initialize=[d for d in list(nw.devices.keys()) if nw.devices[d].active==True 
                                       and nw.devices[c].typ == 'thermal'])
    model.time_set=Set(initialize=list(range(nw.n_t)))

    # Parameters
    model.d = Param(model.bus_set,model.time_set,rule=demand)
    model.lmp = Param(model.time_set,rule=lmps)
    model.p = Param(model.device_set,rule=device_rating)
    model.eta = Param(model.device_set,rule=device_eff)
    model.P = Param(model.bus_set,rule=sub2_lim)
    model.S = Param(rule=sub1_lim)
    model.E = Param(model.device_set,rule=device_consumption)
    model.kappa = Param(rule=slack)
    model.deadline = Param(model.device_set,rule=deadline)

    # Variables
    model.x = Var(model.device_set, model.time_set, within=Boolean)
    model.xi = Var(model.bus_set, model.time_set)
    model.c = Var(model.device_set)
    model.T = Var(model.therm_device_set)
    model.sigma = Var(model.time_set, bounds=(0,1000))
    #model.sigma2 = Var(bounds=(0,1000))

    # Constraints    
    model.xi_sum = Constraint(model.bus_set,rule=xi_sum)
    model.c_def = Constraint(model.device_set,model.time_set,rule=c_def)
    model.c_def2 = Constraint(model.device_set,model.time_set,rule=c_def2)
    model.en_req = Constraint(model.dead_device_set,rule=en_req)
    model.xi0 = Constraint(model.bus_set,rule=xi0)
    model.x0 = Constraint(model.device_set,rule=x0)
    
    for (i,j) in model.device_set:
        if True:#requirements is True:
            for t in range(model.deadline[i,j],nw.n_t):
                 model.x[i,j,t].fix(0)
            '''
            if (nw.devices[i,j].deadline-(1-nw.devices[i,j].x))*model.p[i,j]*nw.t_step > model.E[i,j]:
            #    
            else:
                # minimum charging time
                print('Fully constraining device ('+str(i)+', '+str(j)+')')
                min_time = int(int(model.E[i,j]/(model.p[i,j]*nw.t_step))+1+(1-nw.devices[i,j].x))
                for t in range(min_time,nw.n_t):
                    model.x[i,j,t].fix(0)'''
        else:
            if (nw.devices[i,j].deadline_est-nw.devices[i,j].time_passed)*model.p[i,j]*nw.t_step >  model.E[i,j]:
                for t in range(nw.devices[i,j].deadline_est-nw.devices[i,j].time_passed,nw.n_t):
                    model.x[i,j,t].fix(0)
                    
            # Once our estimate of deadline is exceeded, assume vehicle will charge now
            else:
                for t in range(int(model.E[i,j]/(model.p[i,j]*nw.t_step))+2,nw.n_t):
                    model.x[i,j,t].fix(0)
    
    if len(model.device_set) > 0:
        model.sub1 = Constraint(model.time_set,rule=sub1)
        #model.sub2 = Constraint(model.bus_set,model.time_set,rule=sub2)
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model

def MPC_model(device,typ,interruptible,bound='Lower'):
    '''
    This function creates an MILP optimization problem for individual devices
    to optimize their consumption
    '''
    
    def prices(model,t):
        return device.prices[t]
    
    def en_req(model):
        return (sum([model.x[t]*device.p*device.t_step*device.eta for t in model.time_set]) 
                + model.sigma >= device.E)
    def unint(model,t):
        if t == 0:
            return Constriant.Skip
        else:
            return model.x[t] >= model.x[t-1]
    
    def temp(model,t):
        if t == 0:
            return model.T[t] == device.T
        else:
            return model.T[t] == (model.T[t-1] + (model.T[t]-device.out_temp[t])/(device.R*device.C)
                                  -device.eta*model.x[t]*device.p*device.t_step/device.C)
        
    def t_max(model,t):
        return model.T[t] <= device.T_max
    def t_min(model,t):
        return model.T[t] >= device.T_min
    
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
    if typ == 'deadline':
        model.en = Constraint(rule=en_req)
        if interruptible is True:
            model.unint = Constraint(model.time_set,rule=unint)
            
    else:
        model.T = Var(model.time_set)
        model.temp = Constraint(model.time_set,rule=temp)
        if bound == 'Lower':
            model.bound = Constraint(model.time_set,rule=t_min)
        else:
            model.bound = Constraint(model.time_set,rule=t_max)
        
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model

def direct_control_model(nw):
    '''
    This functions creates a MILP optimization model for the bus pricing
    '''
    M = 1e6 # for big M formulation
    
    # parameters
    def demand(model,i,t):
        d = copy.deepcopy(nw.nodes[i].d[t])
        if t == 0:
            for dev in nw.devices:
                if (dev[0] == i and nw.devices[dev].active == True 
                    and nw.devices[dev].econ == False):
                    d += nw.devices[dev].x*nw.devices[dev].p
        return d
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
    model.device_set=Set(initialize=[d for d in list(nw.devices.keys()) 
                                     if (nw.devices[d].active==True and nw.devices[d].econ==True)])
    model.time_set=Set(initialize=list(range(nw.n_t)))

    # Parameters
    model.d = Param(model.bus_set,model.time_set,rule=demand)
    model.lmp = Param(model.time_set,rule=lmps)
    model.p = Param(model.device_set,rule=device_rating)
    model.S = Param(rule=sub1_lim)
    model.E = Param(model.device_set,rule=device_consumption)
    model.xi = Param(model.bus_set, model.time_set,rule=0)
    model.kappa = Param(rule=slack)

    # Variables
    model.x = Var(model.device_set, model.time_set, within=Boolean)
    model.sigma = Var(model.time_set, bounds=(0,1000))

    # Constraints 
    model.en_req = Constraint(model.device_set,rule=en_req)
    
    for (i,j) in model.device_set:
        for t in range(nw.devices[i,j].deadline,nw.n_t):
            model.x[i,j,t].fix(0)
    
    if len(model.device_set) > 0:
        model.sub1 = Constraint(model.time_set,rule=sub1)
        #model.sub2 = Constraint(model.bus_set,model.time_set,rule=sub2)
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model
