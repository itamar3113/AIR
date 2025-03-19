import numpy as np
from motion_planning.motion_executor import MotionExecutor
from mujoco_env.common.ur5e_fk import forward
from mujoco_env.sim_env import SimEnv

# TODO: check the limits  
"""
workspace_x_lims = [-1.0, -0.45]
workspace_y_lims = [-1.0, -0.45]
"""

# Locate an area where both robots can reach
# You can choose any location in the area to place the blocks
def stack_blocks_at_position(x, y, z,blocks):
    z=0
    for block in blocks:
        z += 0.15
        executor.pick_up("ur5e_2",block[0],block[1],block[2]+0.12) 
        executor.plan_and_move_to_xyz_facing_down("ur5e_2", [x, y, z])
        executor.put_down("ur5e_2", x, y, z+ 0.05)
        

block_position = [
    [-0.7, -0.6, 0.03],
    [-0.7, -0.7, 0.03],
    [-0.7, -0.8, 0.03],
    [-0.7, -0.9, 0.03]]

# Create the simulation environment and the executor
env = SimEnv()
executor = MotionExecutor(env)

# Add blocks to the world by enabling the randomize argument and setting the block position in the reset function of the SimEnv class 
env.reset(randomize=False, block_positions=block_position)

# moveJ is utilized when the robot's joints are clear to you but use carefully because there is no planning here
move_to = [1.305356658502026, -0.7908733209856437, 1.4010098471710881, 4.102251451313659, -1.5707962412281837, -0.26543967541515895]
executor.moveJ("ur5e_2", move_to)

# executor.pick_up("ur5e_2", -0.7, -0.6, 0.15)

# executor.plan_and_move_to_xyz_facing_down("ur5e_2", [-0.7, -0.7, 0.15])
# executor.put_down("ur5e_2", -0.7, -0.7, 0.20)
stack_blocks_at_position(-0.6, -0.7, 0.03, block_position)



executor.wait(4)





