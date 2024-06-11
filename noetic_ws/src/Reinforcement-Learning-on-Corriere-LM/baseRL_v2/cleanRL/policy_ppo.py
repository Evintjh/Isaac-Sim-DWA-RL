import torch
import torch.nn as nn
from torch.distributions.normal import Normal
import numpy as np
from torch.nn import functional as F
import math


class CNNBackbone(nn.Module):
    def __init__(self, nframes, envs):
        super(CNNBackbone, self).__init__()
        self.nframes = nframes
        self.obs_size = np.array(envs.observation_space.shape).prod()
        self.act_fea_cv1 = nn.Conv1d(
            in_channels=nframes, out_channels=32, kernel_size=5, stride=2, padding=6, padding_mode='circular'
        )
        # Output sequence length: floor((input sequence length + 2 * padding - kernel_size) / stride) + 1
        self.cnn_output_shape = math.floor((360 + 2*6 -5)/2) + 1
        self.act_fea_cv2 = nn.Conv1d(
            in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=6, padding_mode='circular'
        )
        self.cnn_output_shape = math.floor((self.cnn_output_shape + 2*6 -3)/2) + 1
        self.act_fc1 = nn.Linear(self.cnn_output_shape*32, 256)
        # self.act_fc1 = nn.Linear((int(self.obs_size / 4) + 1 + 7) * 32, 256)
        self.act_fc2 = nn.Linear(256 + (4) * nframes, 128)  # default2, +4 more for WP, +2 for last action
        torch.nn.init.xavier_uniform_(self.act_fc1.weight)
        torch.nn.init.xavier_uniform_(self.act_fc2.weight)
        print('init CNNBackbone')

    def forward(self, feature):
        # print("feature:  ", feature.shape)
        feature = feature.reshape((-1, self.nframes, 364))
        # print("feature:  ", feature.shape)
        x, goal_speed = feature[:, :, :360], feature[:, :, 360:]
        # print("x:", x.shape)
        feat = F.relu(self.act_fea_cv1(x))
        feat = F.relu(self.act_fea_cv2(feat))
        feat = feat.view(feat.shape[0], -1)
        feat = F.relu(self.act_fc1(feat))
        # print(goal_speed.flatten(start_dim=1).shape)
        feat = torch.cat((feat, goal_speed.flatten(start_dim=1)), dim=-1)
        # print(feat.shape)
        feat = F.relu(self.act_fc2(feat))
        return feat


class Agent(nn.Module):

    def __init__(self, envs):
        super(Agent, self).__init__()
        self.backbone_critic = CNNBackbone(3, envs)
        self.backbone_actor = CNNBackbone(3, envs)
        self.crit_fc_value = nn.Linear(128, 1)
        self.act_fc_actor = nn.Linear(128, 2)
        torch.nn.init.xavier_uniform_(self.crit_fc_value.weight)
        torch.nn.init.xavier_uniform_(self.act_fc_actor.weight)

        # print("batch size: ",batch_size)

        self.actor_logstd = nn.Parameter(torch.zeros(1 ,2))
        # self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(envs.action_space.shape)))


    def get_value(self, feature):
        feat = self.backbone_critic(feature)
        feat = self.crit_fc_value(feat)
        return feat

    def get_action_and_value(self, feature, action=None):
        
        
        
        feat = self.backbone_actor(feature)
        action_mean = self.act_fc_actor(feat)

        # act_mean_split = torch.unbind(action_mean, dim=1)
        # # Apply sigmoid activation to the first dimension
        # sigmoid_output = F.sigmoid(act_mean_split[0])
        # # Apply tanh activation to the second dimension
        # tanh_output = F.tanh(act_mean_split[1])
        # # Stack the outputs back together along the second dimension
        # action_mean = torch.stack([sigmoid_output, tanh_output], dim=1)

      

        # print("logstd shape: ",self.actor_logstd.shape,"   action mean shape:",action_mean.shape)
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        else:
            action = action.reshape((-1,2))
        
        # print(f'action : {action}')
        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.get_value(feature)

    def get_eval(self, x):  # TODO clean up
        feat = self.backbone_actor(x)
        action_mean = self.act_fc_actor(feat)
        return action_mean


# class CNNBackbone(nn.Module):
#     def __init__(self, nframes, envs):
#         super(CNNBackbone, self).__init__()
#         self.nframes = nframes
#         self.obs_size = np.array(envs.observation_space.shape).prod()

#         self.act_fc1 = nn.Linear((4) * nframes, 64) 
#         self.act_fc2 = nn.Linear(64, 64)
#         torch.nn.init.xavier_uniform_(self.act_fc1.weight)
#         torch.nn.init.xavier_uniform_(self.act_fc2.weight)
#         print('init CNNBackbone')

#     def forward(self, feature):
#         feature = feature.reshape((feature.size(0), self.nframes, -1))
#         x, goal_speed = feature[:, :, :360], feature[:, :, 360:]
#         feat = F.relu(self.act_fc1(goal_speed))
#         # print(goal_speed.flatten(start_dim=1).shape)
#         # print(feat.shape)
#         feat = F.relu(self.act_fc2(feat))
#         return feat


# class Agent(nn.Module):

#     def __init__(self, nframes, envs):
#         super(Agent, self).__init__()
#         self.backbone_critic = CNNBackbone(nframes, envs)
#         self.backbone_actor = CNNBackbone(nframes, envs)
#         self.act_fc_value = nn.Linear(64, 1)
#         self.act_fc_actor = nn.Linear(64, 2)
#         torch.nn.init.xavier_uniform_(self.act_fc_value.weight)
#         torch.nn.init.xavier_uniform_(self.act_fc_actor.weight)
#         self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(envs.action_space.shape)))


#     def get_value(self, feature):
#         feat = self.backbone_critic(feature)
#         feat = F.relu(self.act_fc_value(feat))
#         return feat

#     def get_action_and_value(self, feature, action=None):
#         feat = self.backbone_actor(feature)
#         action_mean = F.relu(self.act_fc_actor(feat))
#         # print(f'action mean: {action_mean}')
#         # action_mean = self.actor_mean(x)
#         # print("logstd shape: ",self.actor_logstd.shape,"   action mean shape:",action_mean.shape)
#         action_logstd = self.actor_logstd.expand_as(action_mean)
#         action_std = torch.exp(action_logstd)
#         probs = Normal(action_mean, action_std)
#         if action is None:
#             action = probs.sample()
        
#         # print(f'action : {action}')
#         return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.get_value(feature)

#     def get_eval(self, x):  # TODO clean up
#         action_mean = self.actor_mean(x)
#         return action_mean


# def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
#     torch.nn.init.orthogonal_(layer.weight, std)
#     torch.nn.init.constant_(layer.bias, bias_const)
#     return layer

# class Agent(nn.Module):
#     def __init__(self, envs):
#         super().__init__()
#         self.critic = nn.Sequential(
#             layer_init(nn.Linear(np.array(envs.observation_space.shape).prod(), 64)),
#             nn.Tanh(),
#             layer_init(nn.Linear(64, 64)),
#             nn.Tanh(),
#             layer_init(nn.Linear(64, 1), std=1.0),
#         )
#         self.actor_mean = nn.Sequential(
#             layer_init(nn.Linear(np.array(envs.observation_space.shape).prod(), 64)),
#             nn.Tanh(),
#             layer_init(nn.Linear(64, 64)),
#             nn.Tanh(),
#             layer_init(nn.Linear(64, np.prod(envs.action_space.shape)), std=0.01),
#         )
#         self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(envs.action_space.shape)))

#     def get_value(self, x):
#         return self.critic(x)

#     def get_action_and_value(self, x, action=None):
#         action_mean = self.actor_mean(x)
#         action_logstd = self.actor_logstd.expand_as(action_mean)
#         action_std = torch.exp(action_logstd)
#         probs = Normal(action_mean, action_std)
#         if action is None:
#             action = probs.sample()
#         return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(x)

#     def get_eval(self, x):  # TODO clean up
#         action_mean = self.actor_mean(x)
#         return action_mean
