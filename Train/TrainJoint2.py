import sys
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
from GaitAnaylsisToolkit.LearningTools.Trainer import TPGMMTrainer, GMMTrainer
from GaitAnaylsisToolkit.LearningTools.Runner import  GMMRunner, TPGMMRunner
from GaitAnaylsisToolkit.Session import ViconGaitingTrial
from Vicon import Vicon
from GaitCore.Core import Point


frames = {}

frames["stairA"] = [Point.Point(0, 0, 0),
                    Point.Point(63, 0, 0),
                    Point.Point(0, 42, 0),
                    Point.Point(63, 49, 0)]

frames["stairB"] = [Point.Point(0, 0, 0),
                    Point.Point(49, 0, 0),
                    Point.Point(28, 56, 0),
                    Point.Point(70, 70, 0)]

file13 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_02/subject_02_stair_config1_00.csv"
file12 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_01/subject_01 stairconfig1_02.csv"
file11 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_00/subject_00 stairconfig2_00.csv"
file10 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_10/subject_10 stairclimbing_config1_01.csv"
file09 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_09/subject_09 stairclimbing_config1_00.csv"
file07 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_07/subject_07 stairclimbing_config1_00.csv"
file06 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_06/subject_06 stairclimbing_config1_02.csv"
file05 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_05/subject_05_stair_config1_01.csv"
file03 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_03/subject_03_stair_config0_02.csv"
file02 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_02/subject_02_stair_config1_01.csv"
file01 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_01/subject_01 stairconfig1_03.csv"
file00 = "/home/nathaniel/AIM_GaitData/Gaiting_stairs/subject_00/subject_00 stairconfig1_00.csv"


#file_list = [ file1, file2]
file_list = [ file11,file10,file09,file05,file03,file00]

def get_index(files):

    paths = []
    for file in files:
        trial = ViconGaitingTrial.ViconGaitingTrial(vicon_file=file)
        markers = trial.vicon.get_markers()
        markers.smart_sort()
        markers.auto_make_transform(frames)
        hills = trial.get_stairs("LTOE", "stairA")
        paths.append(hills[0])

    return paths

def make_traj(files, hills):

    paths = []
    hips = []
    knees = []
    ankles = []
    for hill, file in zip(hills, files):

        trial = ViconGaitingTrial.ViconGaitingTrial(vicon_file=file)
        markers = trial.vicon.get_markers()
        markers.smart_sort()
        markers.auto_make_transform(frames)
        joints = trial.vicon.get_model_output().get_left_leg()
        hip = []
        knee = []
        ankle = []
        for h in hill:
            hip.append(-(np.pi / 180) * np.array(joints.hip.angle.x[h[0]]))
            knee.append(-(np.pi / 180) * np.array(joints.knee.angle.x[h[0]]))
            ankle.append(-(np.pi / 180) * np.array(joints.ankle.angle.x[h[0]]))

        hips.append(np.array(hip))
        knees.append(np.array(knee))
        ankles.append(np.array(ankle))

    return hips, knees, ankles

hills = get_index(file_list)
hip, knee, ankle = make_traj(file_list, hills)

for p in hip:
    plt.plot(p)

trainer = TPGMMTrainer.TPGMMTrainer([hip], "/home/nathaniel/catkin_ws/src/ambf_walker/config/hip2", 25, 0.01)
trainer.train()
runner = TPGMMRunner.TPGMMRunner("/home/nathaniel/catkin_ws/src/ambf_walker/config/hip2.pickle")
ddx = []
for i in range(runner.get_length()):
    x = runner.step()
    xx = runner.ddx[0]
    ddx.append(xx)
plt.plot(ddx, linewidth=4)

plt.show()