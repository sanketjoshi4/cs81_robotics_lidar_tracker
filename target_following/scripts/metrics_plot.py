#!/usr/bin/env python

import csv
import sys
import matplotlib.pyplot as plt

try:
    file_name = sys.argv[1]
    try:
        with open(file_name) as csv_file:
            output = [row for row in csv.reader(csv_file, delimiter=',')][1:]
            csv_file.close()

            x1 = [d[0] for d in output]
            y1 = [d[1] for d in output]
            plt.plot(x1, y1, label="Dist")
            plt.xlabel('dist (m)')
            plt.ylabel('time (sec)')
            plt.title('Target distance vs time')
            plt.legend()

            export_name = file_name[:-4] + ".png"
            plt.savefig(export_name)
            plt.title('Target dist cs time')
            print 'Exported graph ({})'.format(export_name)
    except:
        print "File not found ({})".format(file_name)
except:
    print "Missing file name argument"
