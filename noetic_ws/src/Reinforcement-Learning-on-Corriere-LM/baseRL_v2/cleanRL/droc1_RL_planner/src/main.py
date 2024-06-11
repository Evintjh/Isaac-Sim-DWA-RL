import os
import datetime
import shutil
import time
from utils import timeit
# from trainer_sac import SACTrainer
from trainer_ppo import PPOTrainer
from omegaconf import OmegaConf
from env_real_world import JackalEnv

from torch.utils.tensorboard import SummaryWriter
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack

import warnings

warnings.filterwarnings("ignore")


def parse_args(args):
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
            env = JackalEnv(headless=True, args=args)
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
    
    print(OmegaConf.to_yaml(args))
    # Train
    env = load_env(args)
    if args.algo == "PPO":
        agent = PPOTrainer(env, args)
    else:
        raise Exception("Algo not found")
    # agent.train()
    print("Training Completed") 

    # Evaluate
    # env = load_env(args, mode='eval')
    results = agent.eval(env, MODEL_PATH)
    print("results", results)


CONFIG_PATH = "/home/justin/src/baseRL_v2/cleanRL/config.yaml"
ENV_PATH = "/home/justin/src/baseRL_v2/cleanRL/env_real_world.py"
#MODEL_PATH = "/home/justin/src/baseRL_v2/cleanRL/Jackal-v0/trained/172644/model/model_995328.pth"  #3obs
MODEL_PATH = "/home/justin/src/baseRL_v2/cleanRL/Jackal-v0/trained/141606/model/model_995328.pth" # single agent random goal
#MODEL_PATH = "/home/justin/src/baseRL_v2/cleanRL/Jackal-v0/trained/random_goal_multiagent_30/model/model_79203840.pth" #multi agent random goal
#MODEL_PATH = "/home/justin/src/baseRL_v2/cleanRL/Jackal-v0/trained/random_goal_random_1_obs/model/model_2703360.pth" #random multi agent one obstacle
DEBUG_MODE = False

if __name__ == "__main__":
    print("%s : %s \n\n" % (os.environ["HOSTNAME"], datetime.datetime.now()))
    main()
