import matplotlib.pyplot as plt

import numpy as np

STRATEGY_TO_LABEL = {
    'TURTLEBOT': 'TURTLEBOT', 'N/A': 'N/A', 'DS_IDLE': 'Bezczynny', 'DS_RESTART': 'Restart',
    'DS_ALONG_WALL_2': 'Jazda wzdluz sciany',
    'DS_PASS_THROUGH_DOOR_3': 'Przejazd przez drzwi (v3)', 'DS_APPROACH_MARKER_2': 'Zblizenie sie do ArUco (v2)',
    'DS_PASS_THROUGH_DOOR_2': 'Przejazd przez drzwi (v2)', 'DS_APPROACH_MARKER': 'Zblizenie sie do ArUco (v1)',
    'DS_TEST_TRACK_ARUCOS': 'Sledzenie ArUco', 'DS_DEAD_RECKONING_TEST': 'Test jazdy zliczeniowej',
    'DS_ACCORDING_TO_MARKER': 'Ustawienie sie wzgledem ArUco', 'DS_STOP_WALL': 'Zatrzymanie przed sciana',
}

STRATEGY_TO_LABEL_IDX = {
    'TURTLEBOT': 4,
    'N/A': 0, 'DS_IDLE': 1, 'DS_RESTART': 2, 'DS_ALONG_WALL_2': 3, 'DS_PASS_THROUGH_DOOR_3': 4,
    'DS_APPROACH_MARKER_2': 5, 'DS_PASS_THROUGH_DOOR_2': 0, 'DS_APPROACH_MARKER': 0,
    'DS_TEST_TRACK_ARUCOS': 0, 'DS_DEAD_RECKONING_TEST': 0, 'DS_ACCORDING_TO_MARKER': 0,
    'DS_STOP_WALL': 0,
}

colors = ['gray', 'black', 'orange', 'green', 'blue', 'red']


class LogEntry:
    def __init__(self, timestamp, position, orientation, linear, angular, feedback, guideline, strategy_name):
        self.timestamp = timestamp
        self.position = position
        self.orientation = orientation
        self.linear = linear
        self.angular = angular
        self.feedback = feedback
        self.guideline = guideline
        self.strategy_name = strategy_name


class Section:
    def __init__(self, label, path_color):
        self._label = label
        self._path_color = path_color
        self._points = []

    def add_point(self, point):
        self._points.append(point)

    def get_path_color(self):
        return self._path_color

    def get_label(self):
        return self._label

    def get_points(self):
        return self._points


class Plotter:
    def __init__(self):
        self.is_turtlebot = False

    @staticmethod
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

    def convert_to_entry(self, entry_string):
        parts = entry_string.split(',')
        timestamp = float(parts[0].replace('T:', ''))
        position = (float(parts[2]), float(parts[3]), float(parts[4]))
        orientation = (float(parts[6]), float(parts[7]), float(parts[8]))
        linear = (float(parts[10]), float(parts[11]), float(parts[12]))
        angular = (float(parts[14]), float(parts[15]), float(parts[16]))
        feedback = entry_string[entry_string.index('FB:'): entry_string.index('GL:')]
        guideline = entry_string[entry_string.index('GL'):]
        strategy_name = self.get_strategy_name(guideline)
        return LogEntry(timestamp, position, orientation, linear, angular, feedback, guideline, strategy_name)

    def get_strategy_name(self, guideline):
        start_token = "guideline_type:"
        end_token = "parameters"
        try:
            return guideline[guideline.index(start_token) + len(start_token): guideline.index(end_token)] \
                .strip() \
                .replace("\"", '')
        except ValueError:
            print 'strategy name not found'
            return 'TURTLEBOT' if self.is_turtlebot else 'N/A'

    @staticmethod
    def get_sections(log_entries):
        drive_sections = []
        drive_section = None
        prev_strategy_name = None

        for log_entry in log_entries:
            if prev_strategy_name != log_entry.strategy_name:
                if drive_section is not None:
                    drive_sections.append(drive_section)
                drive_section = Section(log_entry.strategy_name, colors[STRATEGY_TO_LABEL_IDX[log_entry.strategy_name]])
            prev_strategy_name = log_entry.strategy_name
            drive_section.add_point(log_entry.position)
        drive_sections.append(drive_section)

        return drive_sections

    def plot_path_comparison(self, first_csv, second_scv, plot_file_name, is_turtlebot=False):
        self.is_turtlebot = is_turtlebot
        flattened_logs = self.flatten_log(first_csv)
        entries = [self.convert_to_entry(entry) for entry in flattened_logs]

        flattened_logs_2 = self.flatten_log(second_scv)
        entries_2 = [self.convert_to_entry(entry) for entry in flattened_logs_2]

        sections = self.get_sections(entries)
        sections_2 = self.get_sections(entries_2)

        fig, axs = plt.subplots(2, 2)
        fig.suptitle('Passage without (left) and with (right) the obstacles')

        added_labels = {}

        self.plot_sections(added_labels, axs, sections, subplot_placement=(0, 0))

        self.plot_sections(added_labels, axs, sections_2, subplot_placement=(0, 1))

        axs[1, 0].legend(loc="upper center")

        axs[0, 0].set_aspect('equal')
        axs[0, 1].set_aspect('equal')

        axs[1, 0].set_aspect('equal')
        plt.savefig(plot_file_name, format='svg', dpi=2400)

    @staticmethod
    def plot_sections(added_labels, axs, sections, subplot_placement):
        for section in sections:
            data = section.get_points()
            data = np.array(data)

            if section.get_label() not in added_labels:
                print 'new label %s' % section.get_label()
                added_labels[section.get_label()] = 1
                axs[1, 0].plot((0, 0), color=section.get_path_color(), label=section.get_label())

            axs[subplot_placement].plot(data[:, 0], data[:, 1], color=section.get_path_color(), linewidth=1)


plotter = Plotter()

plotter.plot_path_comparison('data/capo_position2019-08-01 02:58:1564621127.csv',
                             'data/capo_position2019-08-01 03:10:1564621857.csv',
                             'with_and_without_obstacles_2019-08-01.svg')

plotter.plot_path_comparison('data/capo_position2019-08-04 10:53:1564908808.csv',
                             'data/capo_position2019-08-04 11:27:1564910871.csv',
                             'with_and_without_obstacles_2019-08-04.svg')

plotter.plot_path_comparison('data/turtlebot_position2019-08-05 23:39:1565041179.csv',
                             'data/turtlebot_position2019-08-05 23:45:1565041559.csv',
                             'tutlebot_with_and_without_obstacles_2019-08-05.svg', True)
