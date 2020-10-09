import sys
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
from lib.GaitAnalysisToolkit.LearningTools.Trainer import TPGMMTrainer, GMMTrainer
from lib.GaitAnalysisToolkit.LearningTools.Runner import  GMMRunner, TPGMMRunner
from lib.GaitAnalysisToolkit.Session import ViconGaitingTrial as Trial
from lib.GaitAnalysisToolkit.lib.Vicon import Vicon
from lib.GaitAnalysisToolkit.lib.GaitCore.Core import Point



def smooth(y, box_pts):
    box = np.ones(box_pts) / box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth


def get_stair_ranges(file, side="R"):
    trial = Trial.ViconGaitingTrial(vicon_file=file)
    if side == "R":
        m = trial.vicon.markers.get_marker("RTOE")
    else:
        m = trial.vicon.markers.get_marker("LTOE")

    z = []
    for i in range(len(m)):
        z.append(m[i].z)

    N = 10
    z = smooth(map(int, z), 5)
    z = np.convolve(z, np.ones((N,)) / N, mode='valid')

    max_peakind = np.diff(np.sign(np.diff(z))).flatten()  # the one liner
    max_peakind = np.pad(max_peakind, (1, 1), 'constant', constant_values=(0, 0))
    max_peakind = [index for index, value in enumerate(max_peakind) if value == -2]
    secound_step = max_peakind[-1]
    first_step = max_peakind[-2]

    index = secound_step
    while z[index] != z[index + 1]:
        print index
        index += 1
    final_index = index

    index = first_step
    while z[index] != z[index - 1]:
        index -= 1
    start_index = index
    # plt.plot(z)
    return (start_index, final_index)


def compare_stair_angles(files, side):

    hip = []
    knee = []
    ankle = []


    indiecs = {}
    for file, s in zip(files, side):
        rn = get_stair_ranges(file, s)
        indiecs[file] = rn

    for file, s in zip(files, side):
        trial = Trial.ViconGaitingTrial(vicon_file=file)
        if s == "R":
            joints = trial.vicon.get_model_output().get_right_leg()
        else:
            joints = trial.vicon.get_model_output().get_left_leg()
        rn = indiecs[file]
        hip.append(-(np.pi/180)*np.array(joints.hip.angle.x[rn[0]:rn[1]]))
        knee.append(-(np.pi/180)*np.array(joints.knee.angle.x[rn[0]:rn[1]]))
        ankle.append(-(np.pi/180)*np.array(joints.ankle.angle.x[rn[0]:rn[1]]))

    return hip, knee, ankle


# hip, knee, ankle =compare_stair_angles(
#         ["/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_00/subject_00 stairconfig1_01.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_02/subject_02_stair_config1_03.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_03/subject_03_stair_config0_02.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_04/subject_04_stair_config1_00.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_05/subject_05_stair_config1_00.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_06/subject_06 stairclimbing_config1_02.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_07/subject_07 stairclimbing_config1_01.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_08/subject_08_stair_config1_01.csv",
#          "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_10/subject_10 stairclimbing_config1_00.csv"],
#         ["R", "R", "L", "L", "R", "L", "R", "L", "L"],
#         ["subject00", "subject02", "Subject03", "Subject04", "Subject05", "Subject06", "Subject07", "Subject08",
#          "Subject09", "Subject10"])


files =  ["/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_00/subject_00 stairconfig1_01.csv",
         "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_03/subject_03_stair_config0_02.csv",
         "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_04/subject_04_stair_config1_00.csv",
         "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_05/subject_05_stair_config1_00.csv",
         "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_06/subject_06 stairclimbing_config1_02.csv",
         "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_07/subject_07 stairclimbing_config1_01.csv",
         "/home/nathanielgoldfarb/Documents/stairclimbing_data/CSVs/subject_08/subject_08_stair_config1_01.csv" ]

side = ["R",  "L", "L", "R", "L", "R", "R"]
hip, knee, ankle = compare_stair_angles(files, side)



trainer = TPGMMTrainer.TPGMMTrainer(hip, "/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/config/hip.pickle", 25, 0.01)
bic = trainer.train()
runner = TPGMMRunner.TPGMMRunner("/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/config/hip.pickle")
path2 = runner.run()
plt.plot(path2,linewidth=4)

for p in hip:
    plt.plot(p)
plt.legend( ["subject00",  "Subject03", "Subject04", "Subject05", "Subject06", "Subject07", "Subject08"])
plt.show()