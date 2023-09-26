from robot import Robot
from socket_handler import SocketHandler
from threading import Thread
from helper_functions import checksum_fcn
import time


class IRobot(Robot):
    VELOCITY = 1024
    SIM_VELOCITY = 45
    TOLERANCE = 25  # Difference between current and goal state to consider goal reached
    SIM_RATE = 1 / 30  # Rate for publishing to simulator
    COMPARE_RATE = 1 / 30  # Rate for comparing target to current position
    READING_RATE = 1 / 1000  # RATE = 1 / hz

    def __init__(self, normalized=True, virtual=False):
        """
        Interactive Robot is a child of Robot class but contains methods for reading robot states and sending movement
        targets.  Methods operate the same for both virtual and real life robot. Important to remember that real robot
        takes positional goals and virtual robot takes velocity goals.  Because of this the virtual robot needs a thread
        that is always sending targets.

        Args:
            normalized:  If true joint joint_positions is represented via a [0, 1] range else the dynamixel range [0,4095]
            virtual: If true interacting with Unity robot, if false using real robot
        """

        super().__init__(normalized=normalized)  # Call parent class constructor

        self.__socket_handler = SocketHandler(virtual=virtual)
        self.target_position = [2048] * 5  # Targets are always in dyna range
        self.target_velocity = [self.VELOCITY] * 5  # Velocities for real world robot
        self.virtual = virtual
        self.at_target = False

        if virtual:
            self.target_velocity = [self.SIM_VELOCITY] * 5  # Velocities for virtual robot
            self.sim_thread_running = False
            self.sim_thread = Thread(target=self.publish_sim_motor_states)
            # Only used by simulator
            self.target_motor_states = [0] * 5  # 0 Not moving, 1 Increasing, 2 Decreasing

        self.first_packet_read = False
        self.reading_thread_running = False
        self.reading_thread = Thread(target=self.get_joints_from_udp_loop)

        self.target_thread = Thread(target=self.compare_current_to_target)

        # Start the reading thread, it will block until first packet received
        self.start_reading_thread()

    def move_robot(self, joint_positions, wait=False):
        self.update_targets(joint_positions=joint_positions)

        if self.virtual:
            pass  # Packet sending is handled by simulator thread
        else:
            packet = self.build_joints_packet()
            self.__socket_handler.send_packet(packet)
        if wait:
            while self.at_target is False:
                time.sleep(0.01)

    def build_joints_packet(self):
        """
        Builds a bytearray packet of velocities and positions for each motor given the current target position and
        velocites.  If using the real robot the packet will sent/parsed by the BracIOplexus software to create motor
        commands. This packet structure was developed by us, not brachIOplexus.  It is a simple bytearray container two
        0xFF headers followed by two bytes for position and two bytes for velocity in a little endian format and ending
        with checksum.  See README.md for breakdown of packet structure.  If using simulated robot will use a control
        packet defined by Unity software described here https://github.com/BLINCdev/Virtual-Bento/blob/main/UDP_Packet_Structure.pdf

        Returns:
            bytearray: packet to be sent to BrachIOplexus or Unity Simulator via udp

        Todo:
            This yet supports velocity control, just using default values for now
        """

        if self.virtual:
            pass
        else:
            packet = [0xFF, 0xFF, 4 * len(self._joints)]  # Length = 4 bytes (pos + vel) per joint
            for i in range(len(self._joints)):
                # Convert data into little endian bytes
                packet.append(self.target_position[i] & 0xFF)
                packet.append((self.target_position[i] >> 8) & 0xFF)
                packet.append(self.target_velocity[i] & 0xFF)
                packet.append((self.target_velocity[i] >> 8) & 0xFF)
            packet.append(checksum_fcn(packet[2:]))  # Append checksum function to end
            return bytearray(packet)

    def assert_velocities(self, velocities):
        """
        Asserts that given velocities are in proper range
        Args:
            velocities:

        Returns:
            Velocities in a allowable range
        """

        assert (len(velocities) == len(self._joints)), "Invalid velocities length, pass for all 5 _joints"

        if self.virtual:
            pass
        else:
            for i in range(len(self._joints)):
                assert (velocities[i] in range(self._joints[i].velocity_min, self._joints[
                    i].velocity_max)), "Make sure servo velocities are within valid range"

        return velocities

    def convert_assert_joint_positions(self, joint_positions):
        """
        Converts joint positions to dyna range and asserts that they are in proper range

        Args:
            joint_positions: Desired joint positions in dyna or normalized range

        Returns:
            joint positions in dyna range with positional checks done
        """
        assert (len(joint_positions) == len(self._joints)), "Invalid positions length, pass for all 5 _joints"

        if self.normalized:  # [0,1]
            """If normalized, convert to individual motors dyna range"""
            joint_positions = [float(i) for i in joint_positions]
            for i in range(len(self._joints)):
                joint_positions[i] = self._joints[i].normalized_to_dyna_pos_range((joint_positions[
                    i]))  # Converting normalized to dyna range always ensures it's within allowable range
            joint_positions = [int(i) for i in joint_positions]
        else:  # [0,4095]
            """If raw values, assert if in range"""
            joint_positions = [int(i) for i in joint_positions]
            for i in range(len(self._joints)):
                # Check positions
                assert (self._joints[i].position_min <= joint_positions[i] <= self._joints[
                    i].postion_max), "Make sure servo positions are within valid range"
        return joint_positions

    def compare_current_to_target(self):
        """
        Compares target to current position and decided if target is at goal

        Returns:
            None

        """
        while self.reading_thread_running:
            total_difference = 0
            for i in range(len(self._joints)):
                total_difference += abs(self.target_position[i] - self.get_joint_positions()[i])

            if total_difference > (self.TOLERANCE * len(self._joints)):
                self.at_target = False
            else:
                self.at_target = True

            time.sleep(self.COMPARE_RATE)

    def get_joints_from_udp_loop(self):
        """
        While loop that constantly checks for UDP packets from brachIOplexus and updates robot_obj state when received

        Returns:
            None

        """
        self.reading_thread_running = True

        while self.reading_thread_running:
            # Check if packet is available with current joint_positions
            packet = self.__socket_handler.read_packet()

            if not packet:
                continue

            # First packet read
            if self.first_packet_read is False:
                self.first_packet_read = True

            # Update known robot_obj joint_positions using the packet
            self.update_joints_from_packet(packet)  # Read current joint positions

            time.sleep(self.READING_RATE)

    def publish_sim_motor_states(self):
        """
        Thread that publishes a unity packet to move motors in simulator needs to be running constantly cause will only
        move for a short period of time.

        Returns:

        """
        self.sim_thread_running = True

        while self.sim_thread_running:
            packet = self.build_joints_packet()
            self.__socket_handler.send_packet(packet)
            time.sleep(self.SIM_RATE)

    def start_reading_thread(self):
        """
        Starts the reading_thread that receives packets from brachIOPlexus and updates the robots state.  This will
        block until the first packet is read.

        Args:
            socket_handler: A socket handler required for reading state

        Returns:
            None

        """
        print("Starting read packet thread")
        self.reading_thread.start()

        # Make sure nothing else starts until a first packet is read
        while self.first_packet_read is False:
            time.sleep(0.1)
        print("First packet recieved")

        # Start comparison thread
        self.target_thread.start()

    def stop_reading_thread(self):
        self.reading_thread_running = False
        self.reading_thread.join()
        self.target_thread.join()

    def stop_sim_thread(self):
        self.sim_thread_running = False
        self.sim_thread.join()

    def update_joints_from_packet(self, packet):
        """
        Parses a packet and updates each _joints status (Position, Velocity, Load, Temperature).  The incoming packet
        is the same for both virual and non virtual robot.  BrachIOplexus handles sending that packet and sends in same
        format for virtual and non virtual.

        Args:
            packet (bytearray): UDP packet received from brachIOplexus

        Returns:
            None
        """

        for i in range(3, packet[2], 9):
            idx = packet[i] - 1  # Packet has ID (which starts at 1) need index
            # Since you can read values outside allowable range when torque is not enabled, best always clamp the value
            self._joints[idx].position = self._joints[idx].get_clamped_dyna_joint_position(
                position=int.from_bytes(packet[i + 1:i + 3], byteorder='little'))
            self._joints[idx].velocity = int.from_bytes(packet[i + 3:i + 5], byteorder='little')
            self._joints[idx].load = int.from_bytes(packet[i + 5:i + 7], byteorder='little')
            self._joints[idx].temp = packet[i + 7]
            self._joints[idx].state = packet[i + 8]

    def update_targets(self, joint_positions, velocities=None):
        """
        Creates a new target_position state in dyna range

        Args:
            velocities: Target velocities in sim or real range
            joint_positions: List of desired joint positions in normalized or dyna range

        Returns:

        """
        if velocities is None:
            velocities = self.target_velocity

        # Safety check positional targets and convert to dyna range
        self.target_position = self.convert_assert_joint_positions(joint_positions=joint_positions)
        # Safety check that velocity targets are within allowed range
        self.assert_velocities(velocities=velocities)

        # If virtual robot we need to update motor states to reach target
        if self.virtual:
            # Compare current position to target_position position
            # Update joint motor states  0 - off  1 - increase  2 - decrease
            for i in range(len(joint_positions)):
                if abs((joint_positions[i] - self.target_position[i])) < self.TOLERANCE:
                    self.target_motor_states[i] = 0
                elif joint_positions[i] > self.target_position[i]:
                    self.target_motor_states[i] = 2
                else:
                    self.target_motor_states[i] = 1

    def __repr__(self):
        return f'Position: {self.get_joint_positions()}, Target: {self.target_position}, At Target: {self.at_target}'
