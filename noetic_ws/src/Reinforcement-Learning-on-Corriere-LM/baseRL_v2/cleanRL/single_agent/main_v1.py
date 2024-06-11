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

from trainer_ppo_v1 import PPOTrainer
from omegaconf import OmegaConf
from env_v1 import JackalEnv

from torch.utils.tensorboard import SummaryWriter
# from env_setup_utils.monitor import Monitor
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack

import warnings

warnings.filterwarnings("ignore")


def parse_args(args):
    if args.evaluation:
        args.num_agents = 1
        args.physics_dt = 30.0
        args.eval_episodes = 1000

    if DEBUG_MODE:
        args.total_timesteps = 200
        args.learning_starts = 10
        args.write_interval = 100
        args.batch_size = 8
        args.eval_episodes = 2
    return args


def load_env(args, mode="train"):
    n_agents = 1 if mode in ["eval"] else args.num_envs
    if mode == "train":
        print("Env init: %s , %s" % (args.env_id, mode))

    def make_env(rank=0):
        def _init():
            # env = gym.make(args.env_id)
            # env = gym.make('Pendulum-v1')
            env = JackalEnv(args=args)
            print("action space: ", env.action_space.shape)
            env.seed(agent_seed)
            env.action_space.seed(agent_seed)
            env = Monitor(env)
            return env

        agent_seed = int(args.seed * 10 + rank)
        return _init

    env = DummyVecEnv([make_env(rank=rank) for rank in range(n_agents)])
    env = VecFrameStack(env, n_stack=args.num_stacks, channels_order="first")

    return env



@timeit
def main():
    # Load configs
    args = OmegaConf.load(CONFIG_PATH)
    args = parse_args(args)
    envs_name = args.env_id
    output_path = "/isaac-sim/src/baseRL_v2/cleanRL/%s/%s" % (
        "DEBUG" if DEBUG_MODE else envs_name,
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
    # env = JackalEnv(headless=True)
    # print("action space: ",env.action_space.shape)
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


CONFIG_PATH = "/isaac-sim/src/baseRL_v2/cleanRL/single_agent/config_v1.yaml"
ENV_PATH = "/isaac-sim/src/baseRL_v2/cleanRL/single_agent/env_v1.py"
MODEL_PATH = "/isaac-sim/src/baseRL_v2/cleanRL/Jackal-v0/trained/3_fixed_obstacles_fixed_goal/model/model_995328.pth"
USDA_PATH = 'src/baseRL_v2/cleanRL/single_agent/jackal.usda'

DEBUG_MODE = False

if __name__ == "__main__":
    print("%s : %s \n\n" % (os.environ["HOSTNAME"], datetime.datetime.now()))
    main()
