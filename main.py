from armEnv import armEnv
from planner import *

if __name__ == '__main__':
    renders = True
    env = armEnv(renders=renders, goal_pos=[-0.5, 0.7, 1.3], debug=False)
    #planner = RandomTreePlanner(env, visualize=True)
    planner = HeuristicRandomTreePlanner(env, visualize=True)
    traj = planner.plan(slow=False)
    print('Executing Computed Trajectory')
    planner.execute(traj)
    breakpoint()
