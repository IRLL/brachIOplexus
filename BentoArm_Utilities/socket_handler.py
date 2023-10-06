from helper_functions import checksum_fcn
import socket
import select


class SocketHandler:
    def __init__(self, virtual=False, remote=False):
        """
        Socket handler used for UDP communication with BrachIOPlexus uses one port for sending and another for
        receiving.  Due to the dual port nature of this setup it is best to have one reading_thread constantly looking for
        packets and another sending when needed.

        Args:
            virutal: If true will send packets to the Unity port, else, to BrachIOplexus port
            remote: If True will send packets to static IRL laptop ip (192.168.1.69) else localhost

        Attributes:
            port_tx (int): The port that this script will send data to.  Default (30006)
            port_rx (int): The port that this script will receive data from. Default (30007)
            udp_ip (string): The IP address of the computer/program that you want to send data to. Use 127.0.0.1 when communicating between two programs on the same computer.
            bufferSize (int): Buffer size for incoming bytes.  Default (1024)
            sock (socket): UDP socket
            virtual: Whether sending movement packets to real arm, or simulated arm
        """
        if virtual:
            self.port_tx = 30004
        else:
            self.port_tx = 30006
        self.port_rx = 30007
        if remote:
            self.udp_ip = "192.168.1.69"
        else:
            self.udp_ip = "127.0.0.1"
        self.buffer_size = 1024
        self.failed_packets = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(
            ("", self.port_rx))  # Binding to a blank address should look for all possible connections
        self.sock.setblocking(False)  # We don't want RX to block anything

    def read_packet(self):
        """
        Receives a BentoArm joint_positions packet (more about packet structure in google doc guide) , checks the
        checksum, and returns the joint_positions message which contains all the information needed by the Robot object
        """
        result = select.select([self.sock], [], [], 0.0)
        if len(result[0]) == 0:
            return None
        message = result[0][0].recv(self.buffer_size)
        checksum = checksum_fcn(message[2:-1])  # Check checksum (does not include header and checksum in message)
        if message[0] == 0xFF and message[1] == 0xFF and checksum == message[-1]:
            # If header is correct and checksum is passed return the message
            return message
        else:
            self.failed_packets += 1
            return None

    def send_packet(self, packet):
        """Sends the packet, typically a Bento arm packet message"""
        self.sock.sendto(packet, (self.udp_ip, self.port_tx))
