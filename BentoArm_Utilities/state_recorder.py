import csv
from interactive_robot import IRobot
import signal
import datetime
import time


class StateRecorder:
    RATE = 1 / 250  # rate = 1 / hz

    def __init__(self, normalized=True, print_data=False):
        self.data = []
        # For state recorder it does not matter if virtual or not, compare target is made false for better processing
        self.robot = IRobot(normalized=normalized, compare_target=False)  # Create Robot object
        self.print_data = print_data
        if normalized:
            self.filename = datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S_normalized.csv')
        else:
            self.filename = datetime.datetime.now().strftime('%Y_%m_%d_%H%M%S_dynavalues.csv')
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        """Since the program runs continuously in a while True loop you need a signal to handle writing the data to a csv
        and exiting the program once a signal (Ctrl-c) / (Stop Button IDE) is detected"""
        print("Signal Detected. Stopping joint read thread and exiting...")
        self.robot.stop_robot()
        with open(self.filename, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerows(self.data)
        exit(0)

    def main(self):
        input("Press enter to record robot_obj joint_positions, press ctrl-c to finish")
        while True:
            if self.print_data:
                self.robot.print_joints()
            self.data.append(self.robot.get_joint_positions())
            time.sleep(self.RATE)

    def test(self):
        """Records joint_positions for 10 seconds"""
        input(f"Press enter to record robot_obj joint_positions: Normalized={self.robot.normalized}")
        current_time = time.time()
        while time.time() - current_time < 10:
            if self.print_data:
                self.robot.print_joints()
            self.data.append(self.robot.get_joint_positions())
            time.sleep(self.RATE)
        print(f"Recording Time: {time.time() - current_time}")

        self.robot.stop_robot()
        with open(self.filename, 'w', newline='') as fp:
            w = csv.writer(fp)
            w.writerows(self.data)


if __name__ == "__main__":
    # Printing is not recommended as it will eat up alot of the kernel given the rate you get data
    state_record = StateRecorder(normalized=True, print_data=False)
    state_record.main()
