from interactive_robot import IRobot
from inverse_kinematics import InverseKinematics
import time

robot = IRobot(normalized=True)
ik = InverseKinematics(robot_obj=robot)

while True:
    joint_positions = robot.get_joint_positions()
    for joint_pos in joint_positions:
        print(f"{joint_pos:.3f} ", end='')
    print(f"\n {ik.get_end_effector_position_xyz()} \n")
    time.sleep(0.2)

robot.stop_robot()
