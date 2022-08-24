import datetime, random
from components import (DistNetwork, EVCharger)
from sim_platform import (Simulation, Sim_Plot, get_caiso_lmps, get_caiso_en, get_ca_temp)

# available household data
hhs = ['bid_10007.csv','bid_10017.csv','bid_10021.csv',
       'bid_100002.csv','bid_100023.csv','bid_100026.csv',
       'bid_100031.csv','bid_100054.csv','bid_100085.csv',
       'bid_100108.csv','bid_100110.csv','bid_100124.csv',
       'bid_100125.csv','bid_100135.csv','bid_100142.csv',
       'bid_100159.csv','bid_100160.csv','bid_100165.csv',
       'bid_100173.csv','bid_100181.csv','bid_100195.csv',
       'bid_100198.csv','bid_100208.csv','bid_100260.csv',
       'bid_100269.csv','bid_100290.csv','bid_100291.csv',
       'bid_100296.csv','bid_100299.csv','bid_100308.csv',
       'bid_100322.csv','bid_100326.csv','bid_100337.csv',
       'bid_100350.csv','bid_100361.csv','bid_100362.csv',
       'bid_100369.csv','bid_100373.csv','bid_100383.csv',
       'bid_100392.csv','bid_100399.csv','bid_100413.csv',
       'bid_100434.csv','bid_100446.csv','bid_100448.csv',
       'bid_100461.csv','bid_100462.csv','bid_100471.csv',
       'bid_100480.csv','bid_105114.csv']

# available EV data
evs = ['ev_35000617.csv','ev_35000817.csv','ev_35001017.csv',
       'ev_35001217.csv','ev_35001417.csv','ev_35001717.csv',
       'ev_35001817.csv','ev_35002217.csv','ev_35002517.csv',
       'ev_36000317.csv','ev_36000517.csv','ev_36000717.csv',
       'ev_36000817.csv','ev_36000917.csv','ev_36001417.csv',
       'ev_36001817.csv','ev_36002117.csv','ev_37000717.csv',
       'ev_37001017.csv','ev_37001117.csv','ev_37001217.csv',
       'ev_37001417.csv','ev_37001717.csv','ev_37002117.csv',
       'ev_37002317.csv','ev_38000817.csv','ev_38001017.csv',
       'ev_38001317.csv','ev_38001417.csv','ev_38001617.csv',
       'ev_38001817.csv','ev_38002217.csv','ev_38002417.csv',
       'ev_233000117.csv','ev_233000517.csv','ev_233000617.csv',
       'ev_233000717.csv','ev_233001117.csv','ev_233001217.csv',
       'ev_233001817.csv','ev_233001917.csv','ev_233002117.csv',
       'ev_233002217.csv','ev_233002417.csv','ev_233002517.csv',
       'ev_490000217.csv','ev_490000617.csv','ev_490001517.csv',
       'ev_490001617.csv','ev_490001817.csv','ev_490002417.csv',
       'ev_490002517.csv','ev_840000617.csv','ev_840000917.csv',
       'ev_840001117.csv','ev_840001217.csv','ev_840001817.csv',
       'ev_840002317.csv','ev_1269000417.csv','ev_1269000517.csv',
       'ev_1269000717.csv','ev_1353000617.csv','ev_1353002417.csv',
       'ev_1353002517.csv','ev_1353004017.csv','ev_2911000817.csv',
       'ev_2911001617.csv','ev_2911002517.csv','ev_2911002617.csv',
       'ev_4441000317.csv']

lmps = get_caiso_lmps(datetime.datetime(2021,1,1),
                      datetime.datetime(2021,2,1),resolution=5)

lmp_est = get_caiso_en(datetime.datetime(2021,1,1),
                       datetime.datetime(2021,2,1),resolution=5)
T_out,GHI = get_ca_temp(datetime.datetime(2020,1,1),
                        datetime.datetime(2020,2,1),resolution=5)


# create a 1 bus distribution network at transmission node #?
network = DistNetwork(30,lmps,lmp_est,30.)
# add households to each node
for node_id in range(network.n_bus):
    for n in range(1):
        network.add_building(node_id,'Building'+str(n),'data/'+hhs[node_id],
                             datetime.datetime(2018,1,1),T_out,GHI,
                             heating=False,cooling=False)

'''        
choices = ['econ','now']
ev_no = 0
# add EV chargers to each node
for node_id in range(network.n_bus):
    for n in range(1):
        network.add_EV(node_id,'Building'+str(n),'EV'+str(n),'data/'+evs[ev_no],
                       datetime.datetime(2018,1,1),choice=choices[node_id%2])
        ev_no += 1
        if ev_no == len(evs):
            ev_no = 0
'''

# add HVACs to each building
for node_id in range(network.n_bus):
    for n in range(1):
        network.add_HVAC(node_id,'Building'+str(n),'HVAC'+str(n),20,25)
        

# intialize simulation
sim = Simulation(network)
sim.run_simulation(24*12,results_filepath='results/sim2',opt=False)
#sim.run_simulation(2592,results_filepath='results/30EVs_1perNode_halfsmart_proposed2')
#sim.run_simulation(2592,results_filepath='results/30EVs_1perNode_allsmart_proposed2')
#sim.run_simulation(2592,results_filepath='results/30EVs_1perNode_halfuncontrolled',opt=False)
#sim.run_simulation(2592,results_filepath='results/30EVs_1perNode_chargenow',opt=False)
#sim.run_benchmark(2592,results_filepath='results/30EVs_1perNode_halfsmart_direct')




#plt = Sim_Plot(network,xstart=432,xend=1296,ystart=0,yend=120,nh=6)
#plt.plot_simulation('results/30EVs_1perNode_chargenow','Uncontrolled',0,'#440154')
#plt.plot_simulation('results/30EVs_1perNode_uncontrolled','Top-down',1,'#31688e')
#plt.plot_simulation('results/30EVs_1perNode_allsmart_proposed','Proposed',2,'#35b779')
#plt.plot_simulation('results/30EVs_1perNode_allsmart_direct','Direct',3,'#fde725')

#plt.save_plot('half_smart.png')

#plt.plot_individuals('Top-down','Proposed','Direct','individuals.png')