#!/usr/bin/env python

import csv
import time


class Metrics:

    def __init__(self):
        self.dists = []  # A list of tuple (timestamp, dist from target)
        self.lost_found = [(0, 0, 0)]  # A list of tuple (count, found count, lost count)

    def feed(self, timestamp, dist=None):
        found = dist is not None
        if found:
            self.dists.append((timestamp, dist))

        cum_lost_found = self.lost_found[-1]
        self.lost_found.append((
            cum_lost_found[0] + 1,
            cum_lost_found[0] + (1 if found else 0),
            cum_lost_found[0] + (0 if found else 1)
        ))

    def generate(self, tag=None):

        print "Simulation ended, exporting metrics..."
        tag = tag if tag is not None else time.strftime("%Y-%m-%d_%H-%M-%S")
        file_name = 'metrics/metrics_dists_{}.csv'.format(tag)

        with open(file_name, mode='w') as metrics_file:
            employee_writer = csv.writer(metrics_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for dist_row in self.dists:
                employee_writer.writerow([dist_row[0], dist_row[1]])
            metrics_file.close()

        print "Metrics exported @ {}".format(file_name)
