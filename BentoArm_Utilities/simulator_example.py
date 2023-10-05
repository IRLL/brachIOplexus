from interactive_robot import IRobot

robot = IRobot(virtual=True, normalized=False)
robot.move_robot([3073, 2570, 3073, 3328, 2800], wait=False)
print("Arrived at target")