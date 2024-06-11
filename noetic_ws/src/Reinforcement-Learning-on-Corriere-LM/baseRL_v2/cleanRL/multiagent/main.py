import sys
import os

# Get the current directory (main_directory/script.py's directory)
current_dir = os.path.dirname(os.path.abspath(__file__))

# Get the parent directory (main_directory)
parent_dir = os.path.dirname(current_dir)

# Add the parent directory to sys.path
sys.path.append(parent_dir)

import os
import datetime
import shutil
import time
import gym
from utils import timeit
from trainer_ppo import PPOTrainer
from omegaconf import OmegaConf
from env_2 import JackalEnv

from torch.utils.tensorboard import SummaryWriter
from env_setup_utils.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack

import warnings

warnings.filterwarnings("ignore")


def parse_args(args):
    if args.evaluation:
        args.num_agents = 1
        args.physics_dt = 30.0
        args.eval_episodes = 1000

    if DEBUG_MODE:
        args.total_timesteps = 320 #num_agents * num_step
        args.num_steps = 5 
        args.eval_episodes = 1
        args.num_minibatches = args.num_steps
        args.eval_interval = 1
        args.timeout = 10
    return args


def load_env(args, mode="train"):
    n_agents = 1 if mode in ["eval"] else args.num_envs
    if mode == "train":
        print("Env init: %s , %s" % (args.env_id, mode))
    return JackalEnv(args=args)


@timeit
def main():
    # Load configs
    args = OmegaConf.load(CONFIG_PATH)
    args = parse_args(args)
    envs_name = args.env_id
    output_path = "/isaac-sim/Reinforcement-Learning-on-Corriere-LM/baseRL_v2/cleanRL/%s/%s" % (
        envs_name,
        time.strftime("%Y%m%d/") + "%06d" % (3 * round(int(time.strftime("%H%M%S")) / 3)),
    )
    os.makedirs(output_path, exist_ok=True)
    os.makedirs(output_path + "/model", exist_ok=True)
    shutil.copy2(CONFIG_PATH, output_path)
    shutil.copy2(ENV_PATH, output_path)
    shutil.copy2(USDA_PATH, output_path)
    print(OmegaConf.to_yaml(args))

    # Main
    os.makedirs(output_path, exist_ok=True)
    writer = SummaryWriter(f"{output_path}/tb/{envs_name}")
    writer.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n%s" % ("\n".join([f"|{key}|{value}|" for key, value in vars(args).items()])),
    )

    # Train
    env = load_env(args)
    print("Starting: ", output_path)
    if args.algo == "PPO":
        agent = PPOTrainer(env, args, output_path, writer)
    else:
        raise Exception("Algo not found")
    if args.evaluation == False:
        agent.train()
    print("Training Completed")

    # Evaluate
    # env = load_env(args, mode='eval')
    results = agent.eval(env, model_path=MODEL_PATH)
    print("results", results)


CONFIG_PATH = "/isaac-sim/Reinforcement-Learning-on-Corriere-LM/baseRL_v2/cleanRL/multiagent/config.yaml"
ENV_PATH = "/isaac-sim/Reinforcement-Learning-on-Corriere-LM/baseRL_v2/cleanRL/multiagent/env_2.py"
MODEL_PATH = "" #Model for evaluation
USDA_PATH = '/isaac-sim/Reinforcement-Learning-on-Corriere-LM/baseRL_v2/cleanRL/multiagent/jackal_multiagent.usda'

DEBUG_MODE = False

if __name__ == "__main__":
    print("%s : %s \n\n" % (os.environ["HOSTNAME"], datetime.datetime.now()))
    main()
