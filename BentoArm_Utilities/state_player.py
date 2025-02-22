import csv
from interactive_robot import IRobot
import sys
import os.path
import time

VIRTUAL = False


def playback(file, normalized, virtual=False):
    RATE = 1 / 100  # RATE = 1 / hz
    robot = IRobot(normalized=normalized, virtual=virtual)  # Setup Robot
    input("Press enter to playback csv file, ensure you've enabled Torque On in BracIOplexus Bento Arm Menu")
    done_check = False
    with open(file, mode='r') as fp:
        csv_file = csv.reader(fp)  # Read CSV File
        start_time = time.time()
        for state in csv_file:
            # Initial check to make sure in proper range
            if not done_check:
                if normalized:
                    assert (0 <= float(
                        state[0]) <= 1), "Value read not in normalized range, did you set normalized correctly"
                else:
                    assert (int(float(
                        state[0])) > 1), "Value read not in dyna motor range, did you set normalized correctly?"
                done_check = True
            robot.move(state, wait=False)
            time.sleep(RATE)
        print(f"Playback Time: {time.time() - start_time}")

    robot._stop_reading_thread()
    if robot.virtual:
        robot._stop_sim_thread()
    robot._socket_handler.sock.close()


if __name__ == "__main__":
    if (len(sys.argv)) != 2:
        print(
            "Please provide name of file you wish to playback such as: \npython state_player.py 2023_07_21_123316.csv")
        exit()

    assert (os.path.isfile(sys.argv[1])), f"{sys.argv[1]} Does not exist"  # Check if file given actually exists
    if sys.argv[1].split('_')[-1] == 'dynavalues.csv':
        playback(normalized=False, file=sys.argv[1], virtual=VIRTUAL)
    else:
        playback(normalized=True, file=sys.argv[1], virtual=VIRTUAL)
