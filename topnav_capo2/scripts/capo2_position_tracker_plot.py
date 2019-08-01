import matplotlib.pyplot as plt
import numpy as np


class LogEntry:
    def __init__(self, timestamp, position, orientation, linear, angular, feedback, guideline):
        self.timestamp = timestamp
        self.position = position
        self.orientation = orientation
        self.linear = linear
        self.angular = angular
        self.feedback = feedback
        self.guideline = guideline


def flatten_log(csv_file_name):
    with open(csv_file_name, 'r') as csv:
        flattened = []
        curr_log = None
        for line in csv.readlines():
            if line.startswith('T:'):
                if curr_log is not None:
                    curr_log = curr_log.replace('\n', ' ')
                    flattened.append(curr_log)
                curr_log = line
            else:
                curr_log += line
        return flattened


def convert_to_entry(entry_string):
    parts = entry_string.split(',')
    timestamp = float(parts[0].replace('T:', ''))
    position = (float(parts[2]), float(parts[3]), float(parts[4]))
    orientation = (float(parts[6]), float(parts[7]), float(parts[8]))
    linear = (float(parts[10]), float(parts[11]), float(parts[12]))
    angular = (float(parts[14]), float(parts[15]), float(parts[16]))
    feedback = entry_string[entry_string.index('FB:'): entry_string.index('GL:')]
    guideline = entry_string[entry_string.index('GL'):]
    return LogEntry(timestamp, position, orientation, linear, angular, feedback, guideline)


flattened_logs = flatten_log('data/capo_position2019-08-01 02:58:1564621127.csv')
entries = [convert_to_entry(entry) for entry in flattened_logs]

flattened_logs_2 = flatten_log('data/capo_position2019-08-01 03:10:1564621857.csv')
entries_2 = [convert_to_entry(entry) for entry in flattened_logs_2]

verts = [(entry.position[0], entry.position[1]) for entry in entries]
verts_2 = [(entry.position[0], entry.position[1]) for entry in entries_2]

data = np.array(verts)
data_2 = np.array(verts_2)

fig, axs = plt.subplots(1, 2)
fig.suptitle('Passage without (left) and with (right) the obstacles')

axs[0].plot(data[:, 0], data[:, 1])
axs[1].plot(data_2[:, 0], data_2[:, 1])

axs[0].set_aspect('equal')
axs[1].set_aspect('equal')

# plt.show()

plt.savefig('with_and_without_obstacles.svg', format='svg', dpi=1200)

print 'done'
