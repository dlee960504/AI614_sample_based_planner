import os, inspect
import math

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
os.sys.path.insert(0, currentdir)

import pybullet as p
import pybullet_data
from collections import namedtuple
from attrdict import AttrDict
#from pyRobotiqGripper import *
real = False

def setup_sisbot(p, uid):
    controlJoints = ["shoulder_pan_joint","shoulder_lift_joint",
                     "elbow_joint", "wrist_1_joint",
                     "wrist_2_joint", "wrist_3_joint", 'left_gripper_motor', 'right_gripper_motor']

    jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
    numJoints = p.getNumJoints(uid)
    jointInfo = namedtuple("jointInfo", 
                           ["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity","controllable"])
    joints = AttrDict()
    for i in range(numJoints):
        info = p.getJointInfo(uid, i)
        jointID = info[0]
        jointName = info[1].decode("utf-8")
        jointType = jointTypeList[info[2]]
        jointLowerLimit = info[8]
        jointUpperLimit = info[9]
        jointMaxForce = info[10]
        jointMaxVelocity = info[11]
        controllable = True if jointName in controlJoints else False
        info = jointInfo(jointID,jointName,jointType,jointLowerLimit,
                         jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
        if info.type=="REVOLUTE": # set revolute joint to static
            p.setJointMotorControl2(uid, info.id, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        joints[info.name] = info
    controlRobotiqC2 = False
    mimicParentName = False
    return joints, controlRobotiqC2, controlJoints, mimicParentName



class ur5():

    def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01, vr = False, cid=None):
        # For simplest solution upper one, for PPO down one
        self.robotUrdfPath = "./obj/urdf/real_arm.urdf"
        self.robotStartPos = [-1,0,0]
        self.robotStartOrn = p.getQuaternionFromEuler([1.885,1.786,0.132])

        self.lastJointAngle = None
        self.active = False
        self.timeout = 0
        self.cid = cid

        self.reset()

    def reset(self):
        self.uid = p.loadURDF(os.path.join(os.getcwd(), self.robotUrdfPath), self.robotStartPos, self.robotStartOrn,
                             flags=p.URDF_USE_INERTIA_FROM_FILE, globalScaling=2, physicsClientId=self.cid)
        self.joints, self.controlRobotiqC2, self.controlJoints, self.mimicParentName = setup_sisbot(p, self.uid)
        self.endEffectorIndex = 7 # ee_link
        self.numJoints = p.getNumJoints(self.uid)
        self.active_joint_ids = []
        for i, name in enumerate(self.controlJoints):
            joint = self.joints[name]
            self.active_joint_ids.append(joint.id)

    def getActionDimension(self):
        # if (self.useInverseKinematics):
        #     return len(self.motorIndices)
        return 8  # position x,y,z and ori quat and finger angle

    def getObservationDimension(self):
        return len(self.getObservation())

    def setPosition(self, pos, quat):
        p.resetBasePositionAndOrientation(self.uid,pos,quat, physicsClientId=self.cid)

    def resetJointPoses(self):
                # move to this ideal init point
        self.active = False
        #init_joint_config = [0.15328961509984124, -1.8, -1.5820032364177563, -1.2879050862601897, 1.5824233979484994, 0.19581299859677043, 0.012000000476837159, -0.012000000476837159]
        init_joint_config = [0, -math.pi/2, 0, -math.pi/2, 0, 0, 0, 0]
        #for i in range(0,50000):
        self.action(init_joint_config)
        self.active = True
        self.lastJointAngle = init_joint_config[:-2]


    def getObservation(self):
        observation = []
        state = p.getLinkState(self.uid, self.endEffectorIndex, computeLinkVelocity = 1, computeForwardKinematics=True, physicsClientId=self.cid)
        #print('state',state)
        pos = state[0]
        orn = state[1]
        observation.extend(list(pos))
        observation.extend(list(orn))
        joint_states = p.getJointStates(self.uid, self.active_joint_ids, physicsClientId=self.cid)
        
        joint_positions = list()
        joint_velocities = list()
        

        for joint in joint_states:
            joint_positions.append(joint[0])
            joint_velocities.append(joint[1])
        return joint_positions + joint_velocities + observation


    def action(self, motorCommands):
        poses = []
        indexes = []
        forces = []

        for i, name in enumerate(self.controlJoints):
            joint = self.joints[name]

            poses.append(motorCommands[i])
            indexes.append(joint.id)
            forces.append(joint.maxForce)
        l = len(poses)

        p.setJointMotorControlArray(self.uid, indexes, p.POSITION_CONTROL,
                                    targetPositions=poses, targetVelocities =[0]*l, positionGains = [0.03]*l, forces = forces, physicsClientId=self.cid)

    def move_to(self, target_pose, mode = 'abs', noise = False, clip = False):
        # position: target_pose[:3]
        # orientation: target_pose[3:7]
        # finger_angle: target_pose[7]
        finger_angle=3

        jointPose = list(p.calculateInverseKinematics(self.uid, self.endEffectorIndex, target_pose[:3], target_pose[3:7], physicsClientId=self.cid))
        #jointPose = list(p.calculateInverseKinematics2(self.uid, [self.endEffectorIndex], [target_pose[:3]], physicsClientId=self.cid))
        # print(self.getObservation()[:len(self.controlJoints)]) ## get the current joint positions
        jointPose[7] = -finger_angle/25
        jointPose[6] = finger_angle/25
        self.action(jointPose)
        self.lastJointAngle = jointPose
        return jointPose

