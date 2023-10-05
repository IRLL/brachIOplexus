from inverse_kinematics import InverseKinematics
from interactive_robot import IRobot
import time

### DYNA RANGE EXAMPLES

robot = IRobot(normalized=False, virtual=False)
ik = InverseKinematics(robot_obj=robot)

positions = [2048, 2580, 2048, 1511, 2048]
robot.move_robot(joint_positions=positions, wait=True)

goal = [52.7, 0, 15.55]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='open')
robot.move_robot(joint_positions=positions, wait=True)
print(f'Goal: {goal}  Hand: Open  Positions: {positions}')

time.sleep(5)

goal = [52.7, 0, 19.55]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='closed')
print(f'Goal: {goal}  Hand: Closed  Positions: {positions}')
robot.move_robot(joint_positions=positions, wait=True)

time.sleep(5)

goal = [51, 0, 25]
positions = ik.get_joints_for_goal_xyz(goal, hand_state='mid')
robot.move_robot(joint_positions=positions, wait=True)
print(f'Goal: {goal}  Hand: Mid  Positions: {positions}')
# MAKE SURE YOU CALL THIS ONCE FINISHED!
robot.stop_robot()

### NORMALIZED VALUES EXAMPLES

robot = IRobot(normalized=True, virtual=False)
ik = InverseKinematics(robot_obj=robot)
# In this example robot does not wait till final move
robot.move_robot((0.5,)*5, wait=False)
robot.move_robot((0.6,)*5, wait=False)
robot.move_robot((0.7,)*5, wait=True)
positions = ik.get_joints_for_goal_xyz(goal, hand_state='mid')  # Goal works in normalized range too
robot.move_robot(positions, wait=True)
print(robot)

# MAKE SURE YOU CALL THIS ONCE FINISHED!
robot.stop_robot()