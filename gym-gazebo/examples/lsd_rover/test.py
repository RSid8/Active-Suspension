#!/usr/bin/env python3

import numpy as np
import torch as T
import torch.nn.functional as F
import torch.nn as nn
import torch.optim as optim
import os
import matplotlib.pyplot as plt 
import gym
import gym_gazebo
import time




if __name__ == '__main__':
	
	
	
	env = gym.make('GazeboMarsLsdForce-Lidar-v0')
	
	
	time.sleep(10)
	done=False
	
	while not done:

	   

	    print(env.get_observation())

	 

	    

	
	
		
	

	


		
   
		
