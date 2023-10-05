from socket_handler import SocketHandler
from inverse_kinematics import  InverseKinematics
from interactive_robot import IRobot
import time

robot = IRobot(normalized=False, virtual=True)
ik = InverseKinematics(robot_obj=robot)

positions = [2048,2580,2048,1511,2048]
robot.move_robot(joint_positions=positions, wait=True)

while True:
    time.sleep(1)

goal = [52.7, 0, 15.55]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='open')
print(f'Goal: {goal}  Hand: Open  Positions: {positions}')

time.sleep(5)

goal = [52.7, 0, 19.55]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='closed')
print(f'Goal: {goal}  Hand: Closed  Positions: {positions}')

time.sleep(5)

goal = [51, 0, 25]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='mid')
print(f'Goal: {goal}  Hand: Mid  Positions: {positions}')
