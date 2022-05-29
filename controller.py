import csv
import numpy as np
import matplotlib.pyplot as plt
from pyomo.opt import SolverFactory, SolverManagerFactory
from pyomo.environ import (ConcreteModel, Objective, Param, Var, Set)

# one option: using Pyomo
model = ConcreteModel()
model.bus_set=Set(initialize=list(nw.nodes.keys()))
model.device_set=Set(initialize=list(nw.devices.keys()))
model.time_set=Set(initialize=list(range(nw.n_t)))

# Parameters
model.d = Param(model.bus_set,model.time_set,rule=func)
model.lmp = Param(model.time_set,rule=func)
model.p = Param(model.device_set,rule=func)
model.P = Param(model.bus_set,rule=func)
model.S = Param(rule=func)
model.E = Param(model.device_set,rule=func)

# Variables
model.x = Var(model.device_set, model.time_set, within=Boolean, 
              initialize=0)
model.xi = Var(model.bus_set, model.time_set,intialize=0)
model.c = Var(model.device_set,initialize=0)

