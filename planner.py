from more_itertools import collapse
import numpy as np
import gym
import random
import time
import math
import pybullet as p

class BaseTreeNode():
    def __init__(self, value, prev=None):
        self.value = value
        self.prev = prev

def distance(pos1:list, pos2:list):
    return np.square(np.array(pos1) - np.array(pos2)).sum()

class BaseTreePlanner():
    def __init__(self, environment:gym.Env):
        self.env = environment
        self.root = self.make_root()
        self.nodes = [self.root]
        self.terminated = False
        self.planning = False

    def make_root(self):
        raise NotImplementedError

    def make_new_node(self, value, prev=None):
        node = BaseTreeNode(value, prev)
        return node

    def sample_action(self):
        raise NotImplementedError
    
    def pick_node(self):
        return NotImplementedError

    @staticmethod
    def find_trajectory(last_node):
        traj = []
        curr_node = last_node
        while curr_node.prev is not None:
            traj.append(curr_node.value)
            curr_node = curr_node.prev

        return reversed(traj)

    def execute(self, traj):
        self.env.reset()
        for action in traj:
            terminated = self.env.step(action)
            time.sleep(0.5)

        return terminated

    def plan_loop(self, slow=False, trial=0):
        raise NotImplementedError


    def plan(self, slow=False, max_try=5):
        self.planning = True
        cnt = max_try
        try:
            new_node = self.plan_loop(slow, trial=cnt)
        except Exception:
            print('Unable to plan a trajectory let me restart')
            exit()
        
        print('Finished planning')
        traj = self.find_trajectory(new_node)
        self.planning = False

        return traj

class RandomTreePlanner(BaseTreePlanner):
    def __init__(self, environment:gym.Env, visualize:bool=False):
        super().__init__(environment)
        self.vis = visualize

    def make_root(self):
        # root node has the value of initial configuration
        init_joint_pose = self.env._arm[0].getObservation()
        return self.make_new_node(init_joint_pose)

    def make_new_node(self, value, prev=None):
        node = super().make_new_node(value, prev)
        if (prev is not None) and self.vis and self.planning:
            p.addUserDebugLine(value[-7:-4], prev.value[-7:-4], lineColorRGB=[0,1,0])
            obs = self.env._arm[0].getObservation()[-7:-4]
            p.addUserDebugPoints([obs], pointColorsRGB = [[0,0,1]], pointSize=5, lifeTime=5)

        return node

    def sample_action(self, max_cnt=50):
        node = self.pick_node()
        curr_pose = node.value
        action = self.env.action_space.sample()
        cnt = 0
        while self.collision_check(curr_pose[:8], action):
            if cnt > max_cnt:
                raise ValueError('Failed to sample action within 1000 samples. Please Check Joint Lock Situation.')
            action = self.env.action_space.sample()
            #print(action)
            cnt += 1
        return action, node

    def collision_check(self, curr_pose, action)->bool:
        # True: good to go
        # False: stop
        if action is None:
            return False

        result = self.env.check_collision(curr_pose, action)
        
        return result

    def pick_node(self):
        return random.choice(self.nodes)

    def plan_loop(self, slow=False, trial=0):
        if trial < 0:
            return None
        new_node = None
        while not self.terminated:
            #node = self.pick_node()
            try:
                action, node = self.sample_action()
            except ValueError as e:
                print(e)
                self.resetArm()
                # new_node = None
                # raise ValueError('Action Sampling Failed')
            self.terminated, jointpose = self.env.step(action)
            obs = self.env._arm[0].getObservation()
            new_node = self.make_new_node(obs, prev=node)
            self.nodes.append(new_node)
            
            if slow:
                time.sleep(1)
        
        return new_node

    def resetArm(self):
        for i, cid in enumerate(self.env.cids):
            #self.env._arm[i].resetJointPoses()
            self.env._arm[i].action(np.random.uniform(low=-math.pi/2, high=math.pi/2, size=(8,)))
            for _ in range(self.env.actionRepeat):
                p.stepSimulation(physicsClientId=cid)

    def execute(self, traj):
        self.env.reset()
        for value in traj:
            action = value[:8]
            #terminated = self.env.step(action)
            terminated = self.env._arm[0].action(action)
            time.sleep(0.5)

        return terminated

class HeuristicRandomTreePlanner(RandomTreePlanner):
    def __init__(self, environment:gym.Env, visualize:bool=False):
        super().__init__(environment, visualize)

    def pick_node(self):
        # go through each node to find the minimum heaurstics
        min_node = self.nodes[0]
        goal = self.env.goal_pos
        min_d = distance(goal, min_node.value[-7:-4])
        for node in self.nodes[1:]:
            pos = node.value[-7:-4]
            d = distance(goal, pos)
            if d < min_d:
                min_node = node
                min_d = d

        return min_node

class RRTplanner(RandomTreePlanner):
    def __init__(self, environment:gym.Env, visualize:bool=False):
        super().__init__(environment, visualize)

    def sample_action(self, max_cnt=100):
        action = self.env.action_space.sample()
        node = self.pick_node(action)
        curr_pose = node.value
        cnt = 0
        while self.collision_check(curr_pose[:8], action):
            if cnt > max_cnt:
                raise ValueError('Failed to sample action within 1000 samples. Please Check Joint Lock Situation.')
            action = self.env.action_space.sample()
            #print(action)
            cnt += 1
        return action, node

    def pick_node(self, action):
        # go through each node to find the minimum heaurstics
        min_node = self.nodes[0]
        goal = action[:3]
        min_d = distance(goal, min_node.value[-7:-4])
        for node in self.nodes[1:]:
            pos = node.value[-7:-4]
            d = distance(goal, pos)
            if d < min_d:
                min_node = node
                min_d = d

        return min_node