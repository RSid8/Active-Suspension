import numpy as np
import torch
import torch.nn as nn
import gym
import gym_gazebo
import time
import os
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.cmd_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_checker import check_env

env = make_vec_env('GazeboMarsLsdForce-Lidar-v0', n_envs=2)
check_env(env)

