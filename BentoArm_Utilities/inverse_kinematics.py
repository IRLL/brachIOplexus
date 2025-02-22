from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from mpl_toolkits.mplot3d import Axes3D
from math import pi
import matplotlib.pyplot as plt
from helper_functions import fill_state, get_diff_xyz, change_scale


class InverseKinematics:

    def __init__(self, robot_obj=None):
        """
        This is the chain/urdf of the arm used for forward and inverse kinematics with the final end effector
        being the center of the end of the fixed chopstick.  All measurements are in centimeters and have been measured
        using the solidworks model.

        Args:
            robot_obj (Robot): Robot class which is needed for the joint min and max positions and for reading robot state

        Attributes:
            bento_chain (Chain): IKPy Chain, a collection of links and _joints representing the bento arm
            max_error (float): Total allowable IK lookup error ( goal_position - position calculated by IK )
        """

        if robot_obj is None:
            from robot import Robot
            self.robot = Robot()
        else:
            self.robot = robot_obj

        self.bento_chain = Chain(name='left_arm', links=[
            OriginLink(),
            URDFLink(
                name="base_to_post",
                origin_translation=[0, 0, 32],
                origin_orientation=[0, 0, 0],
                joint_type="fixed"
            ),
            URDFLink(
                name="extender",
                origin_translation=[12, 0, 0],  # Careful this is set by the user
                origin_orientation=[0, 0, 0],
                joint_type="fixed",
            ),
            URDFLink(
                name="shoulder",
                origin_translation=[0, 0, -6.99],
                origin_orientation=[pi, 0, 0],
                joint_type="revolute",
                rotation=[0, 0, 1],
                bounds=(self.robot._get_joint_objects()[0].radians_min, self.robot._get_joint_objects()[0].radians_max)
            ),
            URDFLink(
                name="shoulder_to_elbow",
                origin_translation=[2.9, 0, 12.503],
                origin_orientation=[0, 0, 0],
                joint_type="revolute",
                rotation=[0, 1, 0],
                bounds=(self.robot._get_joint_objects()[1].radians_min, self.robot._get_joint_objects()[1].radians_max)
            ),
            URDFLink(
                name="elbow_to_forearm",
                origin_translation=[15.123, 0, -0.225],
                origin_orientation=[0, 0, 0],
                joint_type="revolute",
                rotation=[1, 0, 0],
                bounds=(self.robot._get_joint_objects()[2].radians_min, self.robot._get_joint_objects()[2].radians_max)
            ),
            URDFLink(
                name="forearm_to_wrist",
                origin_translation=[4.38, 0, 0.065],
                origin_orientation=[0, 0, 0],
                joint_type="revolute",
                rotation=[0, 1, 0],
                bounds=(self.robot._get_joint_objects()[3].radians_min, self.robot._get_joint_objects()[3].radians_max)
            ),
            URDFLink(
                name="wrist_to_hand",
                origin_translation=[8.43, 0, 0.073],
                origin_orientation=[pi, 0, 0],
                joint_type="fixed",
            ),
            URDFLink(
                name="hand_to_finger",
                origin_translation=[10.52, -0.092, 0],
                origin_orientation=[0, 0, 0],
                joint_type="fixed",
            ), ], active_links_mask=[False, False, False, True, True, True, True, False, False])

        # Ax for plotting
        self.ax = plt.figure().add_subplot(111, projection='3d')
        self.ax.axes.set_xlim3d(left=0, right=55)
        self.ax.axes.set_ylim3d(bottom=-22.5, top=22.5)
        self.ax.axes.set_zlim3d(bottom=0, top=55)

        # Safety params
        self.max_error = 0.1

    def plot_state(self, state=[0, 0, 0, 0]):
        """
        Plots the Bento Arm in the given joint_positions in 3D

        Args:
            state (list): List of joint configurations in [-π,π]: [shoulder, elbow, forearm, wrist]

        Returns:
            None

        """
        self.bento_chain.plot(fill_state(state), self.ax)
        plt.show(block=False)

    def forward_kinematics(self, state=[0, 0, 0, 0], matrix=False):
        """
        Does a forward kinematics lookup taking a joint_positions and calculating where the end of the fixed chopstick is in space

        Args:
            state (list): List of joint configurations in [-π,π]: [shoulder, elbow, forearm, wrist]
            matrix (bool): If true end effector position gets returned as a homogenous matrix, else a XYZ tuple

        Returns:
            The end effectors position as homogenous matrix or XYZ tuple
        """
        position = self.bento_chain.forward_kinematics(fill_state(state))
        if matrix:
            return position  # Return homogenous transform matrix
        else:
            return position[0][3], position[1][3], position[2][3]  # Return X Y Z coordinates

    def inverse_kinematics(self, target_position_xyz=(0, 0, 0), plot=False):
        """
        A crude inverse kinematics lookup which estimates the joint positions needed to have the end effector reach a
        target_position position.  Will also check if the sum of the difference between goal and forward_kinematics calculation
        is greater than self.max_error if so will give an input warning.

        Args:
            target_position_xyz (tuple): Target XYZ position in space for end effector to touch
            plot (bool): If true will plot the joint configuration

        Returns:

        """
        joints = self.bento_chain.inverse_kinematics(target_position_xyz)
        forward = self.forward_kinematics(joints)
        difference = []
        for i in range(3):
            difference.append(abs(target_position_xyz[i] - forward[i]))
        if sum(difference) > self.max_error:
            input(f"WARNING POSITION {target_position_xyz} REDUCED TO {forward} PRESS ENTER IF YOUR SURE!!!")
        if plot:
            self.plot_state(joints)
        return joints

    def get_end_effector_position_xyz(self):
        """
        Converts the current joint positions read by the brachIOplexus packet / Robot class and calculates the position
        of the end effector in 3D space (cm)

        Returns:
            Tuple of position (cm) or Matrix

        """
        if self.robot.normalized:
            joints_to_ik = [change_scale(old_min=0,
                                         old_max=1,
                                         new_min=self.robot._get_joint_objects()[i].radians_min,
                                         new_max=self.robot._get_joint_objects()[i].radians_max,
                                         value=self.robot._get_joint_objects()[i].get_normalized_joint_position()) for i
                            in range(4)]
        else:
            joints_to_ik = [change_scale(old_min=self.robot.DYNA_MIN,
                                         old_max=self.robot.DYNA_MAX,
                                         new_min=-pi,
                                         new_max=pi,
                                         value=self.robot._get_joint_objects()[i].position) for i in range(4)]
        return self.forward_kinematics(joints_to_ik, matrix=False)

    def get_joints_for_goal_xyz(self, goal_xyz, hand_state="mid"):
        """
        Builds an action using the desired goal_xyz and hand_state

        Args:
            goal_xyz: XYZ goal position for end effector in cm
            hand_state: possible options include "closed", "mid", and "open"

        Returns:
            A collection of joint positions required by the Robot class

        """
        ik_radians = self.inverse_kinematics(goal_xyz)[3:8]
        ik_radians[-1] = self.robot._hand_states[hand_state]
        # IKPY returns in [-pi, pi], need in [0,4096] or [0,1]
        if self.robot.normalized:
            joints = [change_scale(old_min=self.robot._get_joint_objects()[i].radians_min,
                                   old_max=self.robot._get_joint_objects()[i].radians_max,
                                   new_min=0,
                                   new_max=1,
                                   value=ik_radians[i]) for i in range(5)]
            return joints

        else:
            return [change_scale(-pi, pi, self.robot._joints[0].DYNA_MIN, self.robot._joints[0].DYNA_MAX, i) for i in
                    ik_radians]


def test_ik():
    """
    Basic test_ik that does both forward and backwards kinematics for a collection of manually defined joint positions,
    asserting that the forward and kinematics position for the joint positions are roughly the same

    Returns:
        None
    """
    # Basic test_ik doing both forward and backwards kinematics assuring both give the same value
    positions = [[0, 0, 0, 0], [pi / 2, 0, 0, pi / 2], [-pi / 2, 0, -pi / 2, -pi / 2]]
    ik = InverseKinematics()

    for pos in positions:
        ik.plot_state(pos)
        for_position = ik.forward_kinematics(pos)
        print(f'Forward Kinematics Position: {for_position}')
        print(f'Target: {pos}')
        state = ik.inverse_kinematics(target_position_xyz=for_position)
        print(f'Required State Radians {state[3:7]}')
        inv_position = ik.forward_kinematics(state=state)
        print(f'Forward Kinematics Position: {inv_position}')
        print('-------------------------------------------------------')
        diff = get_diff_xyz(for_position, inv_position)
        assert (diff < 0.1), "INVERSE VS FORWARDS KINEMATICS DIFFERENT, POSSIBLE BUG"
        print(f"Difference between forward and inverse: {diff}")

    plt.show(block=True)


if __name__ == "__main__":
    test_ik()
