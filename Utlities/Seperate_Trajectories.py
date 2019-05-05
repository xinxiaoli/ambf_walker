import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from scipy.signal import find_peaks_cwt
import pandas
file_path = "/home/nathaniel/git/AMBF_Walker/config/joint_data.csv"
gait_data = {}
column = []

gait_data = pd.read_csv(file_path)
hip = np.array(gait_data["RHipAngles"] ).flatten()
start = np.argmax(np.array(gait_data["RHipAngles"] ) >0)
dH = np.gradient(hip)

max_peakind = np.diff(np.sign(np.diff(hip))).flatten() #the one liner
max_peakind = np.pad(max_peakind, (1, 1), 'constant', constant_values=(0, 0))
max_peakind = [index for index, value in enumerate(max_peakind) if value == -2]
print max_peakind
# left = {"hip" :[] ,"knee" :[] ,"ankle" :[]}
# right = {"hip" :[] ,"knee" :[] ,"ankle" :[]}
#
# crossing = 0
# good_data = gait_data["RHipAngles"]
# print  np.fft.fft(good_data)
# zero_crossing = np.where(np.diff(np.sign(good_data)))[0]
# print  zero_crossing
fig, ax = plt.subplots()
for start in xrange(2,len(max_peakind)-2):
    hip = np.array(gait_data["RHipAngles"][max_peakind[start]:max_peakind[start+1]])
    ax.plot(np.arange(len(hip)), hip)
#ax.plot(range(len(dH)),  dH)
#ax.plot(range(len(ankle)), ankle)

ax.set(xlabel='time (s)', ylabel='voltage (mV)',
       title='About as simple as it gets, folks')
ax.grid()
plt.show()