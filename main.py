from armEnv import armEnv
from planner import *
import argparse
import time

def get_args():
    parser = argparse.ArgumentParser(description='Please specify the model (i.e. Random, Heuristic, RRT')
    parser.add_argument('--mode', required=True, help='mode')
    parser.add_argument('--render', type=bool, required=False, default=True, help='Visualize the simulation')
    args = parser.parse_args()

    return args

if __name__ == '__main__':
    args = get_args()
    renders = args.render
    mode = args.mode
    env = armEnv(renders=renders, goal_pos=[0.5, 0.7, 1.3], debug=False)
    if mode == 'random' or mode == 'r':
        planner = RandomTreePlanner(env, visualize=True)
    elif mode == 'heuristic' or mode == 'h':
        planner = HeuristicRandomTreePlanner(env, visualize=True)
    elif mode == 'RRT' or mode == 'rrt':
        planner = RRTplanner(env, visualize=True)
    else:
        print('Wrong mode')
        exit()
    traj = planner.plan(slow=False)
    print('Executing Computed Trajectory')
    planner.execute(traj)
    time.sleep(5)
