from ctypes.wintypes import PHKEY
import gym
from gym import spaces
from gym.utils import seeding
from scenes import *
import numpy as np
import time

def range_coordinate(eye_z=1, proj_fov=80.0):
    return 2 * eye_z * np.tan(math.radians(proj_fov / 2))

class armEnv(gym.Env):
    """
    Custom environment of Grasping ur5 robot arm that follows gym interface
    reference: https://stable-baselines3.readthedocs.io/en/master/guide/custom_env.html
    """
    metadata = {'render.modes': ['human', 'rgb_array']}
    def __init__(self, urdfRoot=pybullet_data.getDataPath(), actionRepeat=60, goal_pos=[0.2,0.2,0.2],
                 isEnableSelfCollision=True, renders=True, arm='move_ur5', vr=False, debug=False):
        # simulator setting
        self.timeStep = 1. / 120.
        self.urdfRoot = urdfRoot
        self.actionRepeat = actionRepeat
        self.isEnableSelfCollision = isEnableSelfCollision
        self.renders = renders
        self.vr = vr
        self.debug = debug
        self.seed()

        # env setting
        self.terminated = False
        self.arm_name = arm
        self.arm_range = 3
        self.min = -0.5 * self.arm_range
        self.max = 0.5 * self.arm_range

        # (x, y, z, roll, pitch, yaw) of end-effector
        self.action_space = spaces.Box(low=np.array([self.min, 0, 0, -math.pi, -math.pi, -math.pi]),
                                       high=np.array([self.max, self.max, self.max, math.pi, math.pi, math.pi]), dtype=np.float32)

        self.goal_pos = goal_pos
        self.arm_init_pose = [0, 0, -0.1555, 0, 0, 0]
        
        if self.renders:
            self.mainClientId = p.connect(p.SHARED_MEMORY)
            if (self.mainClientId < 0):
                self.mainClientId = p.connect(p.GUI)
            if self.vr:
                p.resetSimulation()
                p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        else:
            p.connect(p.DIRECT)
        self.CollisionClientId = p.connect(p.DIRECT)
        self.cids = [self.mainClientId, self.CollisionClientId]

        self.reset()
        if self.vr:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            p.setRealTimeSimulation(1)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        for cid in self.cids:
            p.setPhysicsEngineParameter(numSolverIterations=10, physicsClientId=cid)
            p.setTimeStep(self.timeStep, physicsClientId=cid)

    def reset(self):
        self.terminated = False

        # for collision check, there is a copy of an environment in different physics client. We need to keep duplicate arm in another client.
        # Therefoer, self._arm is a list of arms in different clients
        self._arm = []
        self.envStepCounter = 0
        for cid in self.cids:
            p.resetSimulation(physicsClientId=cid)
            p.setPhysicsEngineParameter(numSolverIterations=150, physicsClientId=cid)
            if not self.vr:
                p.setTimeStep(self.timeStep, physicsClientId=cid)
            scene_construction(cid, self.goal_pos)
            p.setGravity(0, 0, -10, physicsClientId=cid)
            self._arm.append(load_arm_dim_up(self.arm_name, self.arm_init_pose, cid, dim='Z'))
            p.setRealTimeSimulation(1, physicsClientId=cid)
            for _ in range(120):
                p.stepSimulation(physicsClientId=cid)

    def render(self, mode='human', close=False):
        return

    def step(self, action=None):
        """
        moves motors to desired position.
        """
        if action is None:
            action = np.random.uniform(self.min, self.max, size=(6,))
        jointpose = self.step_to(action, abs_rel='abs')
        self.envStepCounter += 1
        self.terminated = self.check_termination()

        return self.terminated, jointpose

    def step_to(self, action, abs_rel='abs', noise=False, clip=False):
        """
        Move robot arm
        """
        #self._arm.resetJointPoses()

        # 8th element is gripper angle
        if type(action) is np.ndarray:
            action = action.tolist()

        pos = action[:3]
        orn = list(p.getQuaternionFromEuler(action[3:]))
        #orn = list(p.getQuaternionFromEuler([0,0,0]))
        target_pose = pos + orn + [0]

        
        for i in range(len(self.cids)):
            motor_poses = self._arm[i].move_to(target_pose, abs_rel, noise, clip)

        for i in range(self.actionRepeat):
            #time.sleep(self.timeStep)
            for cid in self.cids:
                p.stepSimulation(physicsClientId=cid)

        return motor_poses

    def collision_check_step(self, action, abs_rel='abs', noise=False, clip=False):
        if type(action) is list:
            target_pose = action + [0]
        elif type(action) is np.ndarray:
            target_pose = np.append(action, 0)
        else:
            raise AssertionError('Invalid type')
        
        motor_poses = self._arm[1].move_to(target_pose, abs_rel, noise, clip)

        return motor_poses


    def __del__(self):
        for cid in self.cids:
            p.disconnect(physicsClientId=cid)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # close / termination condition
    def check_termination(self, eps=0.05):
        """
        check whether the situation satisfies the termination condition
        
        check the euclidean distance between the end-effector and the goal position
        """
        obs = self._arm[0].getObservation()[-7:]
        pos = obs[:3]
        distance = np.square(np.array(self.goal_pos) - np.array(pos)).sum()
        if self.debug:
            p.addUserDebugLine(pos, self.goal_pos, lineColorRGB=[1,0,0], physicsClientId=self.mainClientId)

        return distance <= eps

    def check_collision(self, curr_pose, action):
        # first store current state
        # if self.envStepCounter == 0:
        #     return False
        
        stateId = p.saveState(self.CollisionClientId)
        # replicate it in the collision check environment
        p.restoreState(stateId=stateId, physicsClientId=self.CollisionClientId)

        # first move end effector to the selected node pose
        self._arm[1].action(curr_pose)
        for i in range(self.actionRepeat):
            p.stepSimulation(physicsClientId=self.CollisionClientId)

        # step simulation
        self.collision_check_step(action)
        for i in range(self.actionRepeat):
            p.stepSimulation(physicsClientId=self.CollisionClientId)
            contacts = p.getContactPoints(physicsClientId=self.CollisionClientId)
            if len(contacts) > 0:
                break

        # revert collision check environment back to current state
        p.restoreState(stateId=stateId, physicsClientId=self.CollisionClientId)
        p.removeState(stateId)
        
        bodys = []
        for c in contacts:
            bodys.append(c[1:3])
        
        print(action)
        result = len(contacts) > 0

        return result

####################################
#        Manipulation
####################################

def set_motorsid(environment):
    observation = environment._arm[0].getObservation()
    xyz = observation[-7:-4]
    ori = p.getEulerFromQuaternion(observation[-4:])
    xin = xyz[0]
    yin = xyz[1]
    zin = xyz[2]
    rin = ori[0]
    pitchin = ori[1]
    yawin = ori[2]
    motorsIds = []
    motorsIds.append(xin)
    motorsIds.append(yin)
    motorsIds.append(zin)
    motorsIds.append(rin)
    motorsIds.append(pitchin)
    motorsIds.append(yawin)
    motorsIds.append(2)
    return motorsIds

def control_individual_motors(environment, arm):
    motorIds = setup_controllable_motors(environment, arm)
    send_commands_to_motor(environment, motorIds)

def setup_controllable_motors(environment, arm):
    possible_range = 3.2  # some seem to go to 3, 2.5 is a good rule of thumb to limit range.
    motorsIds = []

    for tests in range(0, environment._arm[0].numJoints):  # motors

        jointInfo = p.getJointInfo(environment._arm[0].uid, tests)
        # print(jointInfo)
        qIndex = jointInfo[3]
        motorsIds.append(environment._p.addUserDebugParameter("Motor" + str(tests),
                                                                  -possible_range,
                                                                  possible_range,
                                                                  0.0))
    return motorsIds

def send_commands_to_motor(environment, motorIds):
    done = False
    while (not done):
        action = []
        for motorId in motorIds:
            action.append(environment._p.readUserDebugParameter(motorId))
        print(action)
        state, reward, done, info = environment.step(action)
    #environment.terminated = 1
