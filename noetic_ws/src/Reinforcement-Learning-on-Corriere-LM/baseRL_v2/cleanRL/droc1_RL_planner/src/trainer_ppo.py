import time
import numpy as np
import torch
import torch.optim as optim
from policy_ppo import Agent
from trainer import Trainer
import torch.nn as nn
from collections import deque


class PPOTrainer(Trainer):

    def __init__(self, env, args):

        super().__init__(env, args)

        env = self.env
        device = self.device
        args = self.args

        self.agent = Agent(args.num_stacks,env).to(device)
        self.optimizer = optim.Adam(self.agent.parameters(), lr=args.ppo_lr, eps=1e-5)

        # ALGO Logic: Storage setup
        self.obs = torch.zeros((args.num_steps, args.num_envs) + env.observation_space.shape).to(device)
        self.actions = torch.zeros((args.num_steps, args.num_envs) + env.action_space.shape).to(device)
        self.logprobs = torch.zeros((args.num_steps, args.num_envs)).to(device)
        self.rewards = torch.zeros((args.num_steps, args.num_envs)).to(device)
        self.dones = torch.zeros((args.num_steps, args.num_envs)).to(device)
        self.values = torch.zeros((args.num_steps, args.num_envs)).to(device)
        self.returns = None
        self.advantages = None
        self.success_array = deque(maxlen=20)
        self.timeout_array = deque(maxlen=20)
        self.collision_array = deque(maxlen=20)

        self.batch_size = int(args.num_envs * args.num_steps)
        self.minibatch_size = int(self.batch_size // args.num_minibatches)
        self.num_updates = int((args.total_timesteps * args.num_envs) // self.batch_size)
        self.reward_sum = 0
        self.model_save_interval = self.args.model_save_interval
        

    def train(self):

        args = self.args
        device = self.device

        next_obs = self.env.reset()
        next_obs = torch.Tensor(next_obs).to(device)
        next_done = torch.zeros(args.num_envs).to(device)
        global_step = 0
        for update_i in range(1, self.num_updates + 1):

            # Annealing the rate if instructed to do so.
            if args.anneal_lr:
                frac = 1.0 - (update_i - 1.0) / self.num_updates
                lrnow = frac * args.ppo_lr
                self.optimizer.param_groups[0]["lr"] = lrnow

            global_step, next_obs, next_done = self.rollout(global_step, next_obs, next_done)

            self.update(global_step, update_i)
            if global_step >= self.model_save_interval:
                torch.save(self.agent, self.batch_path + f'/model/model_{global_step}' +'.pth')
                results = self.eval(self.env,self.batch_path + f'/model/model_{global_step}' +'.pth',global_step)
                print('results', results)
                self.model_save_interval += self.args.model_save_interval

        torch.save(self.agent, self.batch_path + '/model/model_' + f'/model_{global_step}' +'.pth')
        self.env.close()

    def rollout(self, global_step, next_obs, next_done):

        args = self.args
        device = self.device

        for step in range(0, args.num_steps):
            global_step += 1 * args.num_envs
            

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

            # print("infos: ",infos)

            for idx, d in enumerate(done):
                if d:
                    self.success_array.append(infos[idx]["done_reason"][1] == 1)
                    self.success_rate = np.mean(self.success_array)
                    self.collision_array.append(infos[idx]["done_reason"][1] == 2)
                    self.collision_rate = np.mean(self.collision_array)
                    self.timeout_array.append(infos[idx]["done_reason"][1] == 3)
                    self.timeout_rate = np.mean(self.timeout_array)
                    # print(infos[idx])
                    self.writer.add_scalar('1returns/returns_agent%s' % idx, infos[idx]["episode"]["r"],
                                           global_step)
                    self.writer.add_scalar('1returns/eplen_agent%s' % idx, infos[idx]["episode"]["l"],
                                           global_step)
                    self.writer.add_scalar("Average rewards/step",reward,global_step)
                    self.writer.add_scalar("eval/success_rate",self.success_rate,global_step)
                    self.writer.add_scalar("eval/timeout_rate",self.timeout_rate,global_step)
                    self.writer.add_scalar("eval/collision_rate",self.collision_rate,global_step)

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
        b_logprobs = self.logprobs.reshape(-1)
        b_actions = self.actions.reshape((-1,) + self.env.action_space.shape)
        b_advantages = self.advantages.reshape(-1)
        b_returns = self.returns.reshape(-1)
        b_values = self.values.reshape(-1)

        # Optimizing the policy and value network
        b_inds = np.arange(self.batch_size)
        clipfracs = []
        for epoch in range(args.update_epochs):
            np.random.shuffle(b_inds)
            for start in range(0, self.batch_size, self.minibatch_size):
                end = start + self.minibatch_size
                mb_inds = b_inds[start:end]

                _, newlogprob, entropy, newvalue = self.agent.get_action_and_value(b_obs[mb_inds], b_actions[mb_inds])
                logratio = newlogprob - b_logprobs[mb_inds]
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
                pg_loss1 = -mb_advantages * ratio
                pg_loss2 = -mb_advantages * torch.clamp(ratio, 1 - args.clip_coef, 1 + args.clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()

                # Value loss
                newvalue = newvalue.view(-1)
                if args.clip_vloss:
                    v_loss_unclipped = (newvalue - b_returns[mb_inds]) ** 2
                    v_clipped = b_values[mb_inds] + torch.clamp(
                        newvalue - b_values[mb_inds],
                        -args.clip_coef,
                        args.clip_coef,
                    )
                    v_loss_clipped = (v_clipped - b_returns[mb_inds]) ** 2
                    v_loss_max = torch.max(v_loss_unclipped, v_loss_clipped)
                    v_loss = 0.5 * v_loss_max.mean()
                else:
                    v_loss = 0.5 * ((newvalue - b_returns[mb_inds]) ** 2).mean()

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

    def eval(self, env,model_path=None,global_step=0):
        if model_path is not None:
            checkpoint = torch.load(model_path,map_location=torch.device('cpu'))
            eval_agent = checkpoint

        eval_agent.eval()
        obs = env.reset()
        eval_ep = 0
        ep_reward = []
        while eval_ep < int(self.args.eval_episodes):
            actions = eval_agent.get_eval(torch.Tensor(obs).to(self.device))
            actions = actions.detach().cpu().numpy()
            #print("action:  ",actions)
            next_obs, rewards, dones, infos = env.step(actions)

            if dones:
                eval_ep += 1
                obs = [infos[0]["terminal_observation"]]
                ep_reward.append(infos[0]["episode"]["r"])
                # print("eval record tensorboard")

            else:
                obs = next_obs

        return ep_reward
