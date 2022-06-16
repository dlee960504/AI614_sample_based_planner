import pybullet as p
from ur5 import ur5
import pybullet_data
import math
import os

turn = math.pi/8

urdfRoot=pybullet_data.getDataPath()
up_rot = p.getQuaternionFromEuler([-math.pi/2, math.pi,0]) # the transform from Z-Y axis up. Most meshes are Z up.
def scene_construction(cid, goal_pos=[0.2, 0.2, 0.2]):
	p.setAdditionalSearchPath(urdfRoot)

	planeID = p.loadURDF("plane.urdf", physicsClientId=cid, useMaximalCoordinates=True)
	#wallID = p.loadURDF('obj/obstacles/wall.urdf', useFixedBase=True, basePosition=[0,-9.5,0], globalScaling=0.02)
	obstacle1ID = p.loadURDF('obj/obstacles/obstacle1.urdf', useFixedBase=True, basePosition=[-0.3, 0.5, 0.9], globalScaling=0.007)
	obstacle2ID = p.loadURDF('obj/obstacles/obstacle2.urdf', useFixedBase=True, basePosition=[-0.6, -0.6, 0.3], globalScaling=0.01)
	obstacle3ID = p.loadURDF('obj/obstacles/obstacle2.urdf', useFixedBase=True, basePosition=[0.2, 0.4, 0.3], globalScaling=0.007)
	obstacle4ID = p.loadURDF('obj/obstacles/obstacle2.urdf', useFixedBase=True, basePosition=[0.5, -0.4, 0.9], globalScaling=0.007)
	load_goal_region_ball(goal_pos, cid)
	#load_obstacle_box(position=[-0.2, 0.5, 0.4], dimension=[0.5, 0.3, 0.4], cid=cid)

def load_goal_region_ball(position, cid):
	visualShapeId=p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 0.5], visualFramePosition=[0, 0, 0], physicsClientId=cid)

	p.createMultiBody(baseInertialFramePosition=position, baseVisualShapeIndex=visualShapeId, basePosition=position, useMaximalCoordinates=True, physicsClientId=cid)

def load_obstacle_box(position, dimension, cid):
	visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=dimension, rgbaColor=[1, 1, 0, 1], visualFramePosition=[0, 0, 0], physicsClientId=cid)
	collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=dimension, collisionFramePosition=[0, 0, 0], physicsClientId=cid)

	p.createMultiBody(baseMass=1, baseInertialFramePosition=position, baseCollisionShapeIndex=collisionShapeId,
						baseVisualShapeIndex=visualShapeId, basePosition=position, useMaximalCoordinates=True, physicsClientId=cid)

def load_arm_dim_up(arm, pose, cid, dim = 'Z'):
	_arm = ur5(cid=cid)
	arm_rot =p.getQuaternionFromEuler(pose[3:])
	_arm.setPosition(pose[:3], arm_rot)
	_arm.resetJointPoses()
	return _arm

def motor_move(environment: object, motorsIds: object, abs_rel: object) -> object:
    action=[]
    for motorId in motorsIds:
        action.append(motorId)
	# (X, Y, Z, roll, pitch, yaw, fingerAngle)
    action = action[0:3] + list(p.getQuaternionFromEuler([math.pi/2, 90, 1.5])) + [1.5]
    return action

def load_gripper(cid):
	objects = [p.loadURDF((os.path.join(urdfRoot,"pr2_gripper.urdf")), 0.500000,0.300006,0.700000,-0.000000,-0.000000,-0.000031,1.000000, physicsClientId=cid)]
	pr2_gripper = objects[0]
	print ("pr2_gripper=")
	print (pr2_gripper)

	jointPositions=[ 0.550569, 0.000000, 0.549657, 0.000000]
	for jointIndex in range (p.getNumJoints(pr2_gripper), physicsClientId=cid):
		p.resetJointState(pr2_gripper,jointIndex,jointPositions[jointIndex], physicsClientId=cid)
		
	pr2_cid = p.createConstraint(pr2_gripper,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0.2,0,0],[0.500000,0.300006,0.700000], physicsClientId=cid)
	print ("pr2_cid")
	print (pr2_cid)
	return pr2_gripper, pr2_cid
