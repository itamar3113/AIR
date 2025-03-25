from lab_ur5.manipulation.manipulation_controller import ManipulationController
from lab_ur5.motion_planning.geometry_and_transforms import GeometryAndTransforms
from lab_ur5.motion_planning.motion_planner import MotionPlanner
from lab_ur5.robot_inteface.robots_metadata import ur5e_1, ur5e_2


def stack_blocks_at_position(x, y, z, blocks, executor):
    z = 0
    for block in blocks:
        z += 0.15
        executor.pick_up(block[0], block[1], 0)
        executor.put_down(x, y, 0, z + 0.05)


def cube_transfer(block, executor1, executor2):
    executor2.pick_up(block[0], block[1], 0)
    # face up
    executor2.plan_and_move_to_xyzrz(block[0], block[1] + 0.1, block[2] + 0.22, 0, rotation=[0, 0, 0])
    executor1.plan_and_move_to_xyzrz(block[0], block[1] + 0.1, block[2] + 0.32)
    executor1.moveL_relative([0, 0, -0.1])
    executor1.grasp()
    executor2.release_grasp()
    executor1.moveL_relative([0, 0, 0.1])
    

block_position = [
    [-0.7, -0.6, 0.03],
    [-0.7, -0.7, 0.03],
    [-0.7, -0.8, 0.03],
    [-0.7, -0.9, 0.03]]

motion_planner = MotionPlanner()
gt = GeometryAndTransforms.from_motion_planner(motion_planner)

r1_controller = ManipulationController(ur5e_1["ip"], ur5e_1["name"], motion_planner, gt)
r2_controller = ManipulationController(ur5e_2["ip"], ur5e_2["name"], motion_planner, gt)
r1_controller.speed = 0.3
r1_controller.acceleration = 0.3
r2_controller.speed = 0.3
r2_controller.acceleration = 0.3

stack_blocks_at_position(-0.6, -0.7, 0.03, block_position, r2_controller)

cube_transfer(block_position[0], r1_controller, r2_controller)

