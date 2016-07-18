#! /usr/bin/env python

from __future__ import print_function

import argparse
import os
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import colorsys

try:
    import seaborn as sns #beautify plots (optional)
    has_sns=True
except:
    has_sns=False

from ulog_parser import *

"""
Plot timestamps
"""

parser = argparse.ArgumentParser(description='Plot timestamps')
parser.add_argument('filename', metavar='file.ulg', help='ULog input file')

parser.add_argument('-m', '--message', dest='message',
        help='Message name, default is \'sensor_combined\'', default='sensor_combined')

parser.add_argument('-o', '--output', dest='output_format',
        help='Output format: \'pdf\', \'png\', ... . Shows a live plot if this argument is not given.',
        default='')

args = parser.parse_args()
ulog_file_name = args.filename
message_name = args.message
output_format = args.output_format

msg_filter = [message_name]
ulog = ULog(ulog_file_name, msg_filter)
data = ulog.data_list


# get N distinctive colors
def getColors(N, s=0.8, v=0.9):
    HSV_tuples = [(x*1.0/N, s, v) for x in range(N)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples))
    return colors

if output_format:
    linewidth = 0.8
    mpl.rcParams['legend.fontsize'] = 8
else:
    if has_sns: sns.set_style("whitegrid")
    linewidth = 1.2


if (len(data) == 0):
    print("File {0} does not contain {1} messages!".format(ulog_file_name,
        message_name))
    exit(0)


ax = plt.gca()
#colors = getColors(N*2)
plt.xlabel('Time [s]')
plt.ylabel('diff t')

message_data = data[0]
t = message_data.data['timestamp']

plt.plot(t[1:]/1e6, np.diff(t), '-o', linewidth=linewidth, markersize=3)

#legend = plt.legend()
if output_format:
    #plt.subplots_adjust(0.16, 0.22, 0.96, 0.94)
    filename = message_name
    plt.savefig('{}.{}'.format(filename, output_format))
else:
    # select the pan tool
    thismanager = plt.get_current_fig_manager()
    thismanager.toolbar.pan()

    plt.show()


