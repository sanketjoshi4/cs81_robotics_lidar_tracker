
import csv
import time


class Metrics:
    """
    Responsible for generating target distance metrics
    """

    def __init__(self):
        """
        Constructor
        """

        self.dists = []  # A list of tuple (timestamp, dist from target)

    def feed(self, timestamp, dist=None):
        """
        This records data for a particular tick for metric generation
        @param timestamp: The current timestamp
        @param dist: Distance to target
        """

        self.dists.append([timestamp, 0 if dist is None else dist])

    def generate(self, tag=None):
        """
        This generates a csv file with collected metrics data
        @param tag: A unique identifier
        """

        print "Simulation ended, exporting metrics..."
        tag = tag if tag is not None else time.strftime("%Y-%m-%d_%H-%M-%S")

        file_name = 'metrics/metrics_{}.csv'.format(tag)
        with open(file_name, mode='w') as metrics_file:
            csv_writer = csv.writer(metrics_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for dist_row in self.dists:
                csv_writer.writerow([dist_row[0], dist_row[1]])
            metrics_file.close()

        print "Metrics exported @ {}".format(metrics_file)
