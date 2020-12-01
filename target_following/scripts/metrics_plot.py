#!/usr/bin/env python

import csv
import sys
import matplotlib.pyplot as plt


def generate_graph(cmd_vel_freq=1):
    """
    This reads a csv metrics file and generates graph for the same
    The file path needs to be supplied as the first argument while running this code
    The first column of the csv is the seconds passed since start time and second is target distance in meters
    """

    try:
        run = 1
        for file_name in sys.argv[1:]:
            try:
                # Read csv
                with open(file_name) as csv_file:
                    output = [row for row in csv.reader(csv_file, delimiter=',')][1:]
                    csv_file.close()

                    # Prepare matplotlib graph
                    x = [d[0] for i, d in enumerate(output) if i % cmd_vel_freq == 0]
                    y = [d[1] for i, d in enumerate(output) if i % cmd_vel_freq == 0]
                    plt.plot(x, y, label="Run {}".format(run))
                    run += 1

            except:
                print "File not found ({})".format(file_name)
        plt.xlabel('time (sec)')
        plt.ylabel('dist (m)')
        plt.title('Target distance vs time')
        plt.legend()

        # Export graph
        export_name = sys.argv[1][:-4] + ".png"
        plt.savefig(export_name)
        plt.title('Target dist cs time')
        print 'Exported dist graph ({})'.format(export_name)
    except:
        print "Missing file name argument"


if __name__ == "__main__":
    generate_graph(cmd_vel_freq=10)
