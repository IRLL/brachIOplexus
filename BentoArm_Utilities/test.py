from state_player import playback
from state_recorder import StateRecorder
from inverse_kinematics import test_ik
import csv
import os
import time


def main():
    """Test inverse kinematics"""
    test_ik()

    """Test normalized joint_positions recording and playback"""
    recorder = StateRecorder(normalized=True, print_data=False)
    recorder.test()

    playback(file=recorder.filename, normalized=True)

    input("Now test in the simulator, disconnect from robot and connect to simulator then press enter")
    playback(file=recorder.filename, normalized=True, virtual=True)

    os.remove(recorder.filename)
    print("Now record motion inside the simulator")

    """Test dyna values joint_positions recording and playback"""

    recorder2 = StateRecorder(normalized=False, print_data=False)
    recorder2.test()

    with open(recorder2.filename, 'w', newline='') as fp:
        w = csv.writer(fp)
        w.writerows(recorder2.data)

    playback(file=recorder2.filename, normalized=False, virtual=False)
    os.remove(recorder2.filename)


if __name__ == "__main__":
    main()
