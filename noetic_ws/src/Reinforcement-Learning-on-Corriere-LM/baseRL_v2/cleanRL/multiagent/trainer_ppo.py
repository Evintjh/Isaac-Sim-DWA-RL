import time
import numpy as np
import torch
import torch.optim as optim
from policy_ppo import Agent
from trainer import Trainer
import torch.nn as nn
from collections import deque


class PPOTrainer(Trainer):

    def __init__(self, env, args, batch_path, writer):

        super().__init__(env, args, batch_path, writer)

        env = self.env
        device = self.device
        args = self.args

        self.agent = Agent(env).to(device)
        # torch.save(self.agent, self.batch_path + f'/model/model_test' +'.pth')
        # self.agent = Agent(env).to(device)
        self.optimizer = optim.Adam(self.agent.parameters(), lr=args.ppo_lr, eps=1e-5)

        # ALGO Logic: Storage setup
        self.obs = torch.zeros((args.num_steps,env.observation_space.shape[0],env.observation_space.shape[1],env.observation_space.shape[2])).to(device)
        print("obs:",self.obs.shape)
        self.actions = torch.zeros((args.num_steps ,env.action_space.shape[0],env.action_space.shape[1])).to(device)
        print("actions:",self.actions.shape)
        self.logprobs = torch.zeros((args.num_steps, args.num_agents)).to(device)
        self.rewards = torch.zeros((args.num_steps, args.num_agents)).to(device)
        self.dones = torch.zeros((args.num_steps, args.num_agents)).to(device)
        self.values = torch.zeros((args.num_steps, args.num_agents)).to(device)
        self.returns = None
        self.advantages = None
        self.success_array = np.empty((args.num_agents,),dtype=object)
        self.timeout_array = np.empty((args.num_agents,),dtype=object)
        self.collision_array = np.empty((args.num_agents,),dtype=object)
        for i in range(args.num_agents):
            self.success_array[i] = deque(maxlen=20)
            self.timeout_array[i] = deque(maxlen=20)
            self.collision_array[i] = deque(maxlen=20)

        self.batch_size = int(args.num_steps) #horizon length
        self.minibatch_size = int(self.batch_size // args.num_minibatches)
        self.num_updates = int((args.total_timesteps / args.num_agents) // self.batch_size)
        self.reward_sum = 0
        self.model_save_interval = self.args.model_save_interval
        self.eval_interval = self.args.eval_interval
        self.total_eval_ep = 0
        

    def train(self):

        args = self.args
        device = self.device

        next_obs = self.env.reset()
        next_obs = torch.Tensor(next_obs).to(device)
        next_done = torch.zeros(args.num_agents).to(device)
        global_step = 0
        for update_i in range(1, self.num_updates + 1):

            # Annealing the rate if instructed to do so.
            if args.anneal_lr:
                frac = 1.0 - (update_i - 1.0) / self.num_updates
                lrnow = frac * args.ppo_lr
                self.optimizer.param_groups[0]["lr"] = lrnow

            global_step, next_obs, next_done = self.rollout(global_step, next_obs, next_done)

            self.update(global_step, update_i)
            if update_i % self.model_save_interval==0:
                torch.save(self.agent, self.batch_path + f'/model/model_{global_step}' +'.pth')
                self.model_save_interval += self.args.model_save_interval

            if update_i % self.eval_interval == 0:
                results = self.eval(self.env,agent=self.agent,global_step=global_step)
                print('results', results)
                self.eval_interval += self.args.eval_interval

        torch.save(self.agent, self.batch_path + '/model/' + f'/model_{global_step}_last' +'.pth')
        self.env.close()

    def rollout(self, global_step, next_obs, next_done):

        args = self.args
        device = self.device

        for step in range(0, args.num_steps):
            global_step += 1 * args.num_agents
            

            self.obs[step] = next_obs
            self.dones[step] = next_done

            # ALGO LOGIC: action logic
            with torch.no_grad():
                action, logprob, _, value = self.agent.get_action_and_value(next_obs)
                self.values[step] = value.flatten()
            self.actions[step] = action
            self.logprobs[step] = logprob

            # TRY NOT TO MODIFY: execute the game and log data.
       
            next_obs, reward, done, infos = self.env.step(action.cpu().numpy())
            self.reward_sum += np.mean(reward)
            self.rewards[step] = torch.tensor(reward).to(device).view(-1)
            next_obs, next_done = torch.Tensor(next_obs).to(device), torch.Tensor(done).to(device)
            for idx, d in enumerate(done):
                if d:
                    self.success_array[idx].append(infos[idx]["done_reason"][1] == 1)
                    self.success_rate = np.mean(self.success_array[idx])
                    self.collision_array[idx].append(infos[idx]["done_reason"][1] == 2)
                    self.collision_rate = np.mean(self.collision_array[idx])
                    self.timeout_array[idx].append(infos[idx]["done_reason"][1] == 3)
                    self.timeout_rate = np.mean(self.timeout_array[idx])
                    self.writer.add_scalar('Episode_returns/Agent_%s' % idx, infos[idx]["episode"]["r"],
                                            global_step)
                    self.writer.add_scalar('Episode_length/Agent_%s' % idx, infos[idx]["episode"]["l"],
                                            global_step)
                    self.writer.add_scalar("Success_rate/Agent_%s" % idx,self.success_rate,global_step)
                    self.writer.add_scalar("Timeout_rate/Agent_%s" % idx,self.timeout_rate,global_step)
                    self.writer.add_scalar("Collision_rate/Agent_%s" % idx,self.collision_rate,global_step)

                    self.env.reset(idx)

        # bootstrap value if not done
        with torch.no_grad():
            next_value = self.agent.get_value(next_obs).reshape(1, -1)
            advantages = torch.zeros_like(self.rewards).to(device)
            lastgaelam = 0
            for t in reversed(range(args.num_steps)):
                if t == args.num_steps - 1:
                    nextnonterminal = 1.0 - next_done
                    nextvalues = next_value
                else:
                    nextnonterminal = 1.0 - self.dones[t + 1]
                    nextvalues = self.values[t + 1]
                delta = self.rewards[t] + args.gamma * nextvalues * nextnonterminal - self.values[t]
                advantages[
                    t] = lastgaelam = delta + args.gamma * args.gae_lambda * nextnonterminal * lastgaelam
            returns = advantages + self.values

            self.returns = returns
            self.advantages = advantages

        return global_step, next_obs, next_done

    def update(self, global_step, update_i):
        print("update_i",update_i)

        args = self.args
        writer = self.writer

        # flatten the batch
        b_obs = self.obs.reshape((-1,) + self.env.observation_space.shape)
        b_logprobs = self.logprobs
        b_actions = self.actions.reshape((-1,) + self.env.action_space.shape)
        b_advantages = self.advantages
        b_returns = self.returns
        b_values = self.values

        # Optimizing the policy and value network
        b_inds = np.arange(self.batch_size)
        clipfracs = []
        for epoch in range(args.update_epochs):
            np.random.shuffle(b_inds)
            for start in range(0, self.batch_size, self.minibatch_size):
                end = start + self.minibatch_size
                mb_inds = b_inds[start:end]
                _, newlogprob, entropy, newvalue = self.agent.get_action_and_value(b_obs[mb_inds], b_actions[mb_inds])
                logratio = newlogprob - b_logprobs[mb_inds].reshape(-1)
                ratio = logratio.exp()

                with torch.no_grad():
                    # calculate approx_kl http://joschu.net/blog/kl-approx.html
                    old_approx_kl = (-logratio).mean()
                    approx_kl = ((ratio - 1) - logratio).mean()
                    clipfracs += [((ratio - 1.0).abs() > args.clip_coef).float().mean().item()]

                mb_advantages = b_advantages[mb_inds]
                if args.norm_adv:
                    mb_advantages = (mb_advantages - mb_advantages.mean()) / (mb_advantages.std() + 1e-8)
                # Policy loss
                pg_loss1 = -mb_advantages.reshape(-1) * ratio
                pg_loss2 = -mb_advantages.reshape(-1) * torch.clamp(ratio, 1 - args.clip_coef, 1 + args.clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()

                # Value loss
                newvalue = newvalue.view(-1)
                if args.clip_vloss:
                    v_loss_unclipped = (newvalue - b_returns[mb_inds].reshape(-1)) ** 2
                    v_clipped = b_values[mb_inds].reshape(-1) + torch.clamp(
                        newvalue - b_values[mb_inds].reshape(-1),
                        -args.clip_coef,
                        args.clip_coef,
                    )
                    v_loss_clipped = (v_clipped - b_returns[mb_inds].reshape(-1)) ** 2
                    v_loss_max = torch.max(v_loss_unclipped, v_loss_clipped)
                    v_loss = 0.5 * v_loss_max.mean()
                else:
                    v_loss = 0.5 * ((newvalue - b_returns[mb_inds].reshape(-1)) ** 2).mean()

                entropy_loss = entropy.mean()
                loss = pg_loss - args.ent_coef * entropy_loss + v_loss * args.vf_coef

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.agent.parameters(), args.max_grad_norm)
                self.optimizer.step()

            if args.target_kl is not None:
                if approx_kl > args.target_kl:
                    break

        y_pred, y_true = b_values.cpu().numpy(), b_returns.cpu().numpy()
        var_y = np.var(y_true)
        explained_var = np.nan if var_y == 0 else 1 - np.var(y_true - y_pred) / var_y

        # TRY NOT TO MODIFY: record rewards for plotting purposes
        writer.add_scalar("charts/learning_rate", self.optimizer.param_groups[0]["lr"], global_step)
        writer.add_scalar("losses/value_loss", v_loss.item(), global_step)
        writer.add_scalar("losses/policy_loss", pg_loss.item(), global_step)
        writer.add_scalar("losses/entropy", entropy_loss.item(), global_step)
        writer.add_scalar("losses/old_approx_kl", old_approx_kl.item(), global_step)
        writer.add_scalar("losses/approx_kl", approx_kl.item(), global_step)
        writer.add_scalar("losses/clipfrac", np.mean(clipfracs), global_step)
        writer.add_scalar("losses/explained_variance", explained_var, global_step)

        fps = int(global_step / (time.time() - self.start_time))
        writer.add_scalar("zothers/FPS", fps, global_step)
        if update_i % args.ppo_print_interval == 0 or update_i == self.num_updates:
            print("Step %d / %d:, Average returns: %.2f, with %d FPS" % (
                update_i, self.num_updates, self.reward_sum / (args.num_steps * args.ppo_print_interval), fps))
            self.reward_sum = 0

    def eval(self, env,model_path=None,agent=None,global_step=0):
        env._my_world.set_simulation_dt(1.0/env.args.eval_physics_dt)
        print("evaluation")
        if model_path is not None:
            checkpoint = torch.load(model_path)
            eval_agent = checkpoint
        else:
            eval_agent = agent
        

        env.reset(0)
        obs = env.reset()
        eval_ep = 0
        ep_reward = np.zeros((self.args.num_agents))
        success_array = []
        timeout_array = []
        collision_array = []
        while eval_ep < int(self.args.eval_episodes):
            actions = eval_agent.get_eval(torch.Tensor(obs).to(self.device))
            actions = actions.detach().cpu().numpy()
            
            next_obs, rewards, dones, infos = env.step(actions)

            if dones[0]:
                eval_ep += 1
                self.total_eval_ep += 1

                success_array.append(infos[0]["done_reason"][1] == 1)
                success_rate = np.mean(success_array)
                collision_array.append(infos[0]["done_reason"][1] == 2)
                collision_rate = np.mean(collision_array)
                timeout_array.append(infos[0]["done_reason"][1] == 3)
                timeout_rate = np.mean(timeout_array)
                
                

                self.writer.add_scalar('eval/returns_agent',infos[0]["episode"]["r"],self.total_eval_ep)
                self.writer.add_scalar("eval/Success_rate" ,success_rate,self.total_eval_ep)
                self.writer.add_scalar("eval/Timeout_rate" ,timeout_rate,self.total_eval_ep)
                self.writer.add_scalar("eval/Collision_rate" ,collision_rate,self.total_eval_ep)
                env.reset(0)
                env.reset()
                
                print("eval record tensorboard")
            else:
                obs = next_obs

        env._my_world.set_simulation_dt(1.0/env.args.physics_dt)

        return 0.0