#!/usr/bin/env python

import csv
import time


class Metrics:

    def __init__(self):
        self.dists = []  # A list of tuple (timestamp, dist from target)
        self.lost_found = []  # A list of tuple (timestamp, lost count, count)

    def feed(self, timestamp, dist=None):
        found = dist is not None
        if found:
            self.dists.append([timestamp, dist])

        cum_lost_found = self.lost_found[-1] if len(self.lost_found) > 0 else [0, 0, 0]
        self.lost_found.append([
            timestamp,
            cum_lost_found[1] + (0 if found else 1),
            cum_lost_found[2] + 1
        ])

    def generate(self, tag=None):

        print "Simulation ended, exporting metrics..."
        tag = tag if tag is not None else time.strftime("%Y-%m-%d_%H-%M-%S")

        dists_file_name = 'metrics/metrics_dists_{}.csv'.format(tag)
        with open(dists_file_name, mode='w') as dists_metrics_file:
            csv_writer = csv.writer(dists_metrics_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for dist_row in self.dists:
                csv_writer.writerow([dist_row[0], dist_row[1]])
            dists_metrics_file.close()

        recovery_file_name = 'metrics/metrics_recovery_{}.csv'.format(tag)
        with open(recovery_file_name, mode='w') as recovery_metrics_file:
            csv_writer = csv.writer(recovery_metrics_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for dist_row in self.lost_found:
                csv_writer.writerow([dist_row[0], dist_row[1], dist_row[2]])
            recovery_metrics_file.close()

        print "Metrics exported @ {}, {}".format(dists_metrics_file, recovery_file_name)
