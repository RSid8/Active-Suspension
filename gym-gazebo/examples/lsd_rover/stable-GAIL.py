import numpy as np
import torch
import torch.nn as nn
import gym
import gym_gazebo
import time
import tempfile
import pathlib

import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common import results_plotter
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_checker import check_env
from imitation.algorithms import adversarial, bc
from imitation.util import logger, util

tempdir = tempfile.TemporaryDirectory(prefix="quickstart")
tempdir_path = pathlib.Path(tempdir.name)
print(f"All Tensorboards and logging are being written inside {tempdir_path}/.")

logger.configure(tempdir_path / "GAIL/")
env = gym.make('GazeboMarsLsdForce-Lidar-v0')
check_env(env)
env = Monitor(env, log_dir)
timesteps=100000
start=time.time()
gail_trainer = adversarial.GAIL(
    venv,
    expert_data=transitions,
    expert_batch_size=32,
    gen_algo=PPO("MlpPolicy", venv, verbose=1, n_steps=1024),
)
gail_trainer.train(total_timesteps=timesteps)
end=time.time()
time_taken=end-start
print("Training time: " + time_taken)
