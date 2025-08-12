import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from pyulog import core


def loadUlog(path, topic_list):
    ulog = core.ULog(path)

    topics_df = pd.DataFrame()
    for topic in topic_list:
        topic_data = ulog.get_dataset(topic)
        curr_df = pd.DataFrame.from_dict(topic_data.data)
        if topics_df.empty:
            topics_df = curr_df
        else:
            topics_df = pd.concat([topics_df, curr_df], axis=1)
    return topics_df

# file_path = '/home/jaeyoung/Downloads/fluelatal_coverage.ulg'
file_path = '/home/jaeyoung/Downloads/fluelatal_active.ulg'
topic_list = ["sensor_gps"]

log_df = loadUlog(file_path, topic_list)
satellites_used = log_df["satellites_used"].to_numpy()
horizontal_accuracy = log_df["eph"].to_numpy()
vertical_accuracy = log_df["epv"].to_numpy()
time = log_df["time_utc_usec"].to_numpy()/1e6
start_time = time[0]
time = time - time[0]
hdop = log_df["hdop"].to_numpy()
vdop = log_df["vdop"].to_numpy()

fig = plt.figure('GPS Accuracy', figsize=(7, 4.5))
ax1 = fig.add_subplot(3, 1, 1)
ax1.plot(time, satellites_used)
# ax1.set_xlim([0, _])
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Satellites Used')
ax1.set_yticks([20, 24, 28])
ax1.grid(True)

ax2 = fig.add_subplot(3, 1, 3)
ax2.plot(time, horizontal_accuracy, label='hAcc')
ax2.plot(time, vertical_accuracy, label='vAcc')
ax2.set_ylim([0.0, 0.03])
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Accuracy [m]')
ax2.grid(True)
ax2.legend(loc='lower right', ncol=2)

ax3 = fig.add_subplot(3, 1, 2)
ax3.plot(time, hdop, label='HDOP')
ax3.plot(time, vdop, label='VDOP')
ax3.set_ylim([0.0, 1.5])
ax3.set_xlabel('Time [s]')
ax3.set_ylabel('DOP')
ax3.grid(True)
ax3.legend(loc='lower right', ncol=2)


plt.tight_layout()
plt.show()
