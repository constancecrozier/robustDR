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
    def device_rating(model,i,b,j):
        return nw.devices[i,b,j].p
    def device_eff(model,i,b,j):
        return nw.devices[i,b,j].eta
    def device_consumption(model,i,b,j):
        if requirements is True:
            return nw.devices[i,b,j].E
        return max(model.p[i,b,j]*nw.t_step,nw.devices[i,b,j].E_est-nw.devices[i,b,j].charged,
                   model.p[i,b,j]*nw.t_step*nw.devices[i,b,j].x)
    def sub1_lim(model):
        return nw.S
    def sub2_lim(model,i):
        return nw.nodes[i].P
    def slack(model):
        return 1e8
    def deadline(model,i,b,j):
        # first check whether vehicle is accepting high prices
        if (i,b,j) in model.dead_dev_set:
            if nw.devices[i,b,j].x == 1 and nw.devices[i,b,j].prices[0] > 1e3:
                # return minimum time to charge, will fully constrain device
                return int(model.E[i,b,j]/(model.p[i,b,j]*nw.t_step))+1
            if requirements is True:
                return nw.devices[i,b,j].deadline
            else:
                return nw.devices[i,b,j].deadline_est-nw.devices[i,j].time_passed
        else:
            return nw.n_t+1 
    def alpha(model,i,b,j,t):
        if (i,b,j) in model.dead_dev_set:
            return 1.0
        elif nw.devices[i,b,j].T_min != 0:
            # heating device -- lower bound of concern
            return (1.0+(abs(nw.buildings[i,b].T_out[t]-nw.devices[i,b,j].T_min)*
                         nw.buildings[i,b].iR*nw.buildings[i,b].iC*3600*nw.t_step
                         *(nw.n_t-t-1)))
        else:
            # cooling device -- upper bound this time
            return (1.0+(abs(nw.buildings[i,b].T_out[t]-nw.devices[i,b,j].T_max)*
                         nw.buildings[i,b].iR*nw.buildings[i,b].iC*3600*nw.t_step
                         *(nw.n_t-t-1)))
    
    # === CONSTRAINTS ===
    
    # adders sum to zero
    def xi_sum(model,i):
        return sum([model.xi[i,t] for t in model.time_set]) == 0
    
    # big M rule for price sensitiivty of devices
    def c_def(model,i,b,j,t):
        if t < model.deadline[i,b,j]:
            return (model.lmp[t]+model.xi[i,t] <= model.c[i,b,j]*model.alpha[i,b,j,t] + M*(1-model.x[i,b,j,t]))
        else:
            return Constraint.Skip
        
    def c_def2(model,i,b,j,t):
        if t < model.deadline[i,b,j]:
            return (model.c[i,b,j]*model.alpha[i,b,j,t] <= model.lmp[t]+model.xi[i,t] + M*model.x[i,b,j,t])
        else:
            return Constraint.Skip
    
    # constrain observed price and charging behaviour
    def x0(model,i,b,j):
        return model.x[i,b,j,0] == nw.devices[i,b,j].x
    
    def xi0(model,i):
        # find device at right bus
        chosen = None
        for (i2,b,j) in nw.devices:
            if i2 == i:
                chosen = (i2,b,j)
        return model.xi[i,0] == nw.devices[chosen].prices[0]-nw.lmps[0]
    
    def T0(model,i,b):
        return model.T[i,b,0] == nw.buildings[i,b].T
    
    # energy requirement of the devices
    def en_req(model,i,b,j):
        return (sum([model.x[i,b,j,t] for t in model.time_set])*model.p[i,b,j]*nw.t_step
                >= model.E[i,b,j])
    
    # temperature bounds on buildings
    def temp(model,i,b,t):
        if t == 0:
            return model.T[i,b,t] == nw.buildings[i,b].T
        else:
            thr = ((nw.buildings[i,b].T_out[t-1]-model.T[i,b,t-1])
                   *nw.buildings[i,b].iR*nw.buildings[i,b].iC
                   +nw.buildings[i,b].k*nw.buildings[i,b].GHI[t-1]*nw.buildings[i,b].iC)
            for (i,b,j) in connected_to[i,b]:
                thr += (model.p[i,b,j]*model.x[i,b,j,t-1]
                        *nw.devices[i,b,j].eta*1000*nw.buildings[i,b].iC)
            return model.T[i,b,t] == model.T[i,b,t-1] + thr*3600*nw.t_step
    def t_lower(model,i,b,j,t):
        return model.T[i,b,t] >= nw.devices[i,b,j].T_min
    def t_upper(model,i,b,j,t):
        return model.T[i,b,t] <= nw.devices[i,b,j].T_max
    
    # transformer limit
    def sub1(model,t):
        return (sum(model.x[i,b,j,t]*model.p[i,b,j] for (i,b,j) in model.device_set) <=
                model.S - sum(model.d[i,t] for i in model.bus_set) + model.sigma[t])
    
    # objective
    def cost(model):
        c = sum([model.c[i,b,j] for (i,b,j) in model.device_set])
        for (i,b,j) in model.device_set:
            for t in model.time_set:
                c += model.x[i,b,j,t]*model.p[i,b,j]*model.lmp[t]
            
        for t in model.time_set:
            c += model.sigma[t]*model.kappa
        return c


    model = ConcreteModel()
    
    # Sets
    model.time_set=Set(initialize=list(range(nw.n_t)))
    model.bus_set=Set(initialize=list(nw.nodes.keys()))
    model.building_set=Set(initialize=[b for b in list(nw.buildings.keys())])
    model.device_set=Set(initialize=[d for d in list(nw.devices.keys()) if nw.devices[d].active==True])
    model.dead_dev_set=Set(initialize=[d for d in list(nw.devices.keys()) if nw.devices[d].active==True 
                                       and nw.devices[d].type == 'deadline'])
    model.therm_dev_set=Set(initialize=[d for d in list(nw.devices.keys()) if nw.devices[d].active==True 
                                       and nw.devices[d].type == 'thermal'])
    
    # so that we can map thermal devices to buldings
    connected_to = {}
    for (i,b) in list(nw.buildings.keys()):
        connected_to[i,b] = []
    for d in model.therm_dev_set:
        connected_to[d[0],d[1]].append(d)
    

    # Parameters
    model.d = Param(model.bus_set,model.time_set,rule=demand)
    model.lmp = Param(model.time_set,rule=lmps)
    model.p = Param(model.device_set,rule=device_rating)
    model.eta = Param(model.device_set,rule=device_eff)
    model.P = Param(model.bus_set,rule=sub2_lim)
    model.S = Param(rule=sub1_lim)
    model.E = Param(model.dead_dev_set,rule=device_consumption)
    model.kappa = Param(rule=slack)
    model.deadline = Param(model.device_set,rule=deadline)
    model.alpha = Param(model.device_set,model.time_set,rule=alpha)

    # Variables
    model.x = Var(model.device_set, model.time_set, within=Boolean)
    model.xi = Var(model.bus_set, model.time_set,bounds=(0,1e7))
    model.c = Var(model.device_set)
    if len(model.therm_dev_set) > 0:
        model.T = Var(model.building_set,model.time_set)
    model.sigma = Var(model.time_set, bounds=(0,1000))
    #model.sigma2 = Var(bounds=(0,1000))

    # Constraints    
    #model.xi_sum = Constraint(model.bus_set,rule=xi_sum)
    model.c_def = Constraint(model.device_set,model.time_set,rule=c_def)
    model.c_def2 = Constraint(model.device_set,model.time_set,rule=c_def2)
    model.en_req = Constraint(model.dead_dev_set,rule=en_req)
    model.xi0 = Constraint(model.bus_set,rule=xi0)
    model.x0 = Constraint(model.device_set,rule=x0)
    if len(model.therm_dev_set) > 0:
        model.temp = Constraint(model.building_set,model.time_set,rule=temp)
        model.t_bound_l = Constraint(model.therm_dev_set,model.time_set,rule=t_lower)
        model.t_bound_u = Constraint(model.therm_dev_set,model.time_set,rule=t_upper)
        model.T0 = Constraint(model.building_set,rule=T0)
    
    for (i,b,j) in model.dead_dev_set:
        if True:#requirements is True:
            for t in range(model.deadline[i,b,j],nw.n_t):
                 model.x[i,b,j,t].fix(0)
        else:
            if (nw.devices[i,b,j].deadline_est-nw.devices[i,b,j].time_passed)*model.p[i,b,j]*nw.t_step >  model.E[i,b,j]:
                for t in range(nw.devices[i,b,j].deadline_est-nw.devices[i,b,j].time_passed,nw.n_t):
                    model.x[i,b,j,t].fix(0)
                    
            # Once our estimate of deadline is exceeded, assume vehicle will charge now
            else:
                for t in range(int(model.E[i,b,j]/(model.p[i,b,j]*nw.t_step))+2,nw.n_t):
                    model.x[i,b.j,t].fix(0)
    
    if len(model.device_set) > 0:
        model.sub1 = Constraint(model.time_set,rule=sub1)
        #model.sub2 = Constraint(model.bus_set,model.time_set,rule=sub2)
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model

def MPC_model(device,typ,interruptible,building):
    '''
    This function creates an MILP optimization problem for individual devices
    to optimize their consumption
    '''
    print('hi')
    def prices(model,t):
        return device.prices[t]
    
    def en_req(model):
        return (sum([model.x[t]*device.p*device.t_step*device.eta for t in model.time_set]) 
                + model.sigma >= device.E)
    def unint(model,t):
        if t == 0:
            return Constraint.Skip
        else:
            return model.x[t] >= model.x[t-1]
    
    def temp(model,t):
        if t == 0:
            return model.T[t] == building.T
        else:
            return model.T[t] == (model.T[t-1] + 
                                  ((building.T_out[t-1]-model.T[t-1])*building.iR*building.iC
                                   +building.k*building.GHI[t-1]*building.iC
                                   +device.p*model.x[t-1]*device.eta*1000*building.iC)
                                  *3600*device.t_step)
        
    def t_max(model,t):
        if t == 0:
            return Constraint.Skip
        else:
            return model.T[t] <= device.T_max
    def t_min(model,t):
        if t == 0:
            return Constraint.Skip
        else:
            return model.T[t] >= device.T_min
    
    def cost(model):
        #return sum([model.x[t]*model.c[t]*(1-t/len(model.time_set)) for t in model.time_set]) #+ model.sigma*1e8
        return sum([model.x[t]*model.c[t] for t in model.time_set]) #+ model.sigma*1e8
    
    model = ConcreteModel()
    
    # Sets
    model.time_set=Set(initialize=list(range(min(device.deadline,device.n_t))))
    
    # Parameters
    model.c = Param(model.time_set, rule=prices)
    
    # Variables
    model.x = Var(model.time_set, within=Boolean)
    if False:#typ == 'deadline':
        model.sigma = Var(bounds=(0,10))
    else:
        model.sigma = Param(rule=0)
    
    # Constraints
    if typ == 'deadline':
        model.en = Constraint(rule=en_req)
        if interruptible is True:
            model.unint = Constraint(model.time_set,rule=unint)
            
    else:
        model.T = Var(model.time_set)
        model.temp = Constraint(model.time_set,rule=temp)
        model.bound_min = Constraint(model.time_set,rule=t_min)
        model.bound_max = Constraint(model.time_set,rule=t_max)
        
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
    def device_rating(model,i,b,j):
        return nw.devices[i,b,j].p
    def device_consumption(model,i,b,j):
        return nw.devices[i,b,j].E
    def sub1_lim(model):
        return nw.S
    def sub2_lim(model,i):
        return nw.nodes[i].P
    def slack(model):
        return 1e8
    
    # constraints
    def en_req(model,i,j):
        return sum([model.x[i,b,j,t]*model.eta[i,b,j,t] for t in model.time_set])*model.p[i,b,j]*nw.t_step >= model.E[i,b,j]
    def sub1(model,t):
        return (sum(model.x[i,b,j,t]*model.p[i,b,j] for (i,b,j) in model.device_set) <=
                model.S - sum(model.d[i,t] for i in model.bus_set) + model.sigma[t])
    def temp(model,i,b,t):
        if t == 0:
            return model.T[i,b,t] == nw.buildings[i,b].T
        else:
            thr = ((nw.buildings[i,b].T_out[t-1]-model.T[i,b,t-1])
                   *nw.buildings[i,b].iR*nw.buildings[i,b].iC
                   +nw.buildings[i,b].k*nw.buildings[i,b].GHI[t-1]*nw.buildings[i,b].iC)
            for (i,b,j) in connected_to[i,b]:
                thr += (model.p[i,b,j]*model.x[i,b,j,t-1]
                        *nw.devices[i,b,j].eta*1000*nw.buildings[i,b].iC)
            return model.T[i,b,t] == model.T[i,b,t-1] + thr*3600*nw.t_step
    def t_lower(model,i,b,j,t):
        return model.T[i,b,t] >= nw.devices[i,b,j].T_min
    def t_upper(model,i,b,j,t):
        return model.T[i,b,t] <= nw.devices[i,b,j].T_max
    
    # objective
    def cost(model):
        c = 0.
        for (i,b,j) in model.device_set:
            for t in model.time_set:
                c += model.x[i,b,j,t]*model.p[i,b,j]*model.lmp[t]
        for t in model.time_set:
            c += model.sigma[t]*model.kappa
        return c

    model = ConcreteModel()
    
    # Sets
    model.bus_set=Set(initialize=list(nw.nodes.keys()))
    model.device_set=Set(initialize=[d for d in list(nw.devices.keys()) 
                                     if nw.devices[d].active==True])
    model.dead_dev_set=Set(initialize=[d for d in list(nw.devices.keys()) 
                                       if nw.devices[d].active==True 
                                       and nw.devices[d].type == 'deadline'])
    model.therm_dev_set=Set(initialize=[d for d in list(nw.devices.keys()) 
                                        if nw.devices[d].active==True 
                                        and nw.devices[d].type == 'thermal'])
    model.time_set=Set(initialize=list(range(nw.n_t)))
    model.building_set=Set(initialize=[b for b in list(nw.buildings.keys())])
    
    # so that we can map thermal devices to buldings
    connected_to = {}
    for (i,b) in list(nw.buildings.keys()):
        connected_to[i,b] = []
    for d in model.therm_dev_set:
        connected_to[d[0],d[1]].append(d)

    # Parameters
    model.d = Param(model.bus_set,model.time_set,rule=demand)
    model.lmp = Param(model.time_set,rule=lmps)
    model.p = Param(model.device_set,rule=device_rating)
    model.S = Param(rule=sub1_lim)
    model.E = Param(model.dead_dev_set,rule=device_consumption)
    model.xi = Param(model.bus_set, model.time_set,rule=0)
    model.kappa = Param(rule=slack)

    # Variables
    model.x = Var(model.device_set, model.time_set, within=Boolean)
    model.sigma = Var(model.time_set, bounds=(0,1000))
    model.T = Var(model.building_set, model.time_set)

    # Constraints 
    model.en_req = Constraint(model.dead_dev_set,rule=en_req)
    model.temp = Constraint(model.building_set, model.time_set, rule=temp)
    model.t_bound_l = Constraint(model.therm_dev_set, model.time_set, rule=t_lower)
    model.t_bound_u = Constraint(model.therm_dev_set, model.time_set, rule=t_upper)
    
    for (i,b,j) in model.dead_dev_set:
        for t in range(nw.devices[i,j].deadline,nw.n_t):
            model.x[i,b,j,t].fix(0)
    
    if len(model.device_set) > 0:
        model.sub1 = Constraint(model.time_set,rule=sub1)
        #model.sub2 = Constraint(model.bus_set,model.time_set,rule=sub2)
    
    model.Objective = Objective(rule=cost, sense=minimize)
    
    return model
