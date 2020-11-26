#!/usr/bin/env python

import csv
import time


class Metrics:

    def __init__(self):
        self.dists = []  # A list of tuple (timestamp, dist from target)

    def feed(self, timestamp, dist=None):
        self.dists.append([timestamp, 0 if dist is None else dist])

    def generate(self, tag=None):

        print "Simulation ended, exporting metrics..."
        tag = tag if tag is not None else time.strftime("%Y-%m-%d_%H-%M-%S")

        file_name = 'metrics/metrics_{}.csv'.format(tag)
        with open(file_name, mode='w') as metrics_file:
            csv_writer = csv.writer(metrics_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for dist_row in self.dists:
                csv_writer.writerow([dist_row[0], dist_row[1]])
            metrics_file.close()

        print "Metrics exported @ {}".format(metrics_file)
