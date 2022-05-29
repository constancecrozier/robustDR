import csv
import numpy as np
import matplotlib.pyplot as plt

# This wil create an object of a building to add to the grid

class Building:
    def __init__(self,name,base_load,n_dev,t_step):
        '''
        name (str): identifier of individual building
        base_load (str): filename of building characteristics
        n_dev (int): the number of smart devices attached to that building
        t_step:
        '''
        self.name = name
        