#! /usr/bin/env python

"""
Evaluate a ULog file
"""

from __future__ import print_function

import argparse
import os
from matplotlib.cbook import maxdict
import numpy as np
import datetime
import tools as tl

from .core import ULog

#pylint: disable=too-many-locals, invalid-name, consider-using-enumerate

def main():
    """Command line interface"""

    parser = argparse.ArgumentParser(description='Analyze Ulog file')
    parser.add_argument('filename', metavar='file.ulg', help='ULog input file')
    parser.add_argument('-c', '--consumption', action='store_true', help='Turn on consumption calculator')
    parser.add_argument('-w', '--waypoint', action='store_true', help='Waypoint reached time')
    parser.add_argument('-d', '--directory', action='store_true', help='Analyze all files at once')

    args = parser.parse_args()

    ff = bool(True)

    if args.consumption == True:
        ff = False
        ulog = ULog(args.filename, disable_str_exceptions=False)
        consum(ulog)

    if args.waypoint == True:
        ff = False
        ulog = ULog(args.filename, disable_str_exceptions=False)
        waypo(ulog)

    if args.directory == True:
        ff = False
        for curDir, dirs, files in os.walk(args.filename):
            for file in sorted(files):
                ulog = ULog(os.path.join(curDir, file), disable_str_exceptions=False)
                display(ulog)

    if ff == True:
        ulog = ULog(args.filename, disable_str_exceptions=False)
        display(ulog)


def display(ulog):

    t = ft_analyzer(ulog)
    rwto_d = (t[1] - t[0])/1000000

    a1 = round(asp_set(ulog)[2], 1)
    a2 = round(asp_set(ulog)[1], 1)
    a3 = round(asp_set(ulog)[0], 1)
    a4 = t[5]
    a5 = t[6]
    a6 = t[7]
    a7 = t[8]
    a8 = round(rwto_d, 2)
    a9 = round(rwto_max(ulog)[0], 2)
    a10 = round(rwto_max(ulog)[1], 2)
    a11 = round(rwto_max(ulog)[2], 2)
    a16 = round(rwto_max(ulog)[3], 2)
    a12 = round(co_max_roll(ulog), 2)
    a13 = round(roll_integ(ulog), 2)
    a14 = round(pitch_integ(ulog), 2)
    a15 = round(yaw_integ(ulog), 2)
    a16 = round(bat_analyzer(ulog)[0], 2)
    a17 = round(bat_analyzer(ulog)[1], 2)
    a18 = round(bat_analyzer(ulog)[2], 2)

    print(a1, a2, a3, a16, a17, a4, a5, a6, a7, a8, a18, a9, a10, a11, a16, a12, a13, a14, a15)

"""

    print('Min airspped  (param)              :', '{:.1f}'.format(asp_set(ulog)[2]))
    print('Airspeed trim (param)              :', '{:.1f}'.format(asp_set(ulog)[1]))
    print('Max airspped  (param)              :', '{:.1f}'.format(asp_set(ulog)[0]))

    print('Went off path abort                :', t[5])
    print('Roll angle abort                   :', t[6])
    print('Pitch angle abort                  :', t[7])
    print('Pitch angular rate abort           :', t[8])

#    m1, s1 = divmod(int(t[4]/1e6), 60)
#    h1, m1 = divmod(m1, 60)
#    print("Landing on slope : {:d}:{:02d}:{:02d}".format(h1, m1, s1))

    print('Taking off duration                =', '{:.2f}'.format(rwto_d))

    print('Max roll  angle        during rwto :', '{:.2f}'.format(rwto_max(ulog)[0]))
    print('Max pitch angle        during rwto :', '{:.2f}'.format(rwto_max(ulog)[1]))
    print('Max pitch angular rate during rwto :', '{:.2f}'.format(rwto_max(ulog)[2]))
    print('Max roll angle during climbing out :', '{:.2f}'.format(co_max_roll(ulog)))
    print('Max roll  angular rate integral    :', '{:.2f}'.format(roll_integ(ulog)))
    print('Max pitch angular rate integral    :', '{:.2f}'.format(pitch_integ(ulog)))
    print('Max yaw   angular rate integral    :', '{:.2f}'.format(yaw_integ(ulog)))

"""


def roll_integ(ulog):

    """
    Roll angular rate integral evaluator
    """

    d = ulog.get_dataset('rate_ctrl_status')

    rateint = 0
    maxint = 0
    absmaxint = 0

    for i in range(len(d.data['timestamp'])):
        rateint = d.data['rollspeed_integ'][i] * 100
        if absmaxint < abs(rateint):
            absmaxint = abs(rateint)
            maxint = rateint

    return maxint


def pitch_integ(ulog):

    """
    Pitch angular rate integral evaluator
    """

    d = ulog.get_dataset('rate_ctrl_status')

    rateint = 0
    maxint = 0
    absmaxint = 0

    for i in range(len(d.data['timestamp'])):
        rateint = d.data['pitchspeed_integ'][i] * 100
        if absmaxint < abs(rateint):
            absmaxint = abs(rateint)
            maxint = rateint

    return maxint


def yaw_integ(ulog):

    """
    Yaw angular rate integral evaluator
    """

    d = ulog.get_dataset('rate_ctrl_status')

    rateint = 0
    maxint = 0
    absmaxint = 0

    for i in range(len(d.data['timestamp'])):
        rateint = d.data['yawspeed_integ'][i] * 100
        if absmaxint < abs(rateint):
            absmaxint = abs(rateint)
            maxint = rateint

    return maxint


def roll_error(ulog):

    """
    Roll error mean calculator
    !!!Under refurbishment!!!
    """

    d1 = ulog.get_dataset('vehicle_attitude_setpoint')
    d2 = ulog.get_dataset('vehicle_attitude')

    # use same field order as in the log, except for the timestamp
    data_keys1 = [f.field_name for f in d1.field_data]
    data_keys1.remove('timestamp')
    data_keys1.insert(0, 'timestamp') # we want timestamp at first position

    data_keys2 = [f.field_name for f in d2.field_data]
    data_keys2.remove('timestamp')
    data_keys2.insert(0, 'timestamp')

    anglelist = np.empty([0, 3])

    for i in range(len(d1.data['timestamp'])):
        anglelist = np.append(anglelist, [[d1.data[data_keys1[0]][i], d1.data[data_keys1[1]][i], 1]], axis=0)

    for i in range(len(d2.data['timestamp'])):
        anglelist = np.append(anglelist, [[d2.data[data_keys2[0]][i], d2.data[data_keys2[1]][i], 2]], axis=0)

    col_num = 0
    anglelist = anglelist[np.argsort(anglelist[:, col_num])]

    print(len(anglelist))


def waypo(ulog):

    d = ulog.get_dataset('mission_result')

    minsec = []

    for i in range(len(d.data['timestamp'])):
        td = datetime.timedelta(microseconds=int(d.data['timestamp'][i]))
        m, s = divmod(td.seconds, 60)
        ctime = format(m, '02') + ':' + format(s, '02')
        minsec.append(ctime)

    print('time  -  No.')

    for i in range(len(d.data['timestamp'])):
        wayr = d.data['seq_reached'][i]
        print(minsec[i],'-', '{:>3}'.format(wayr))


def consum(ulog):

    """
    Consumption average calculator
    """

    d = ulog.get_dataset('battery_status')

    sm, ss = input('Start time? (format = xx:yy) : ').split(':')
    em, es = input('End time? (format = xx:yy) : ').split(':')

    td = datetime.timedelta(minutes=int(sm), seconds=int(ss))
    st = int(td.total_seconds()*1000000)
    td = datetime.timedelta(minutes=int(em), seconds=int(es))
    et = int(td.total_seconds()*1000000)

    for i in range(len(d.data['timestamp'])):
        if d.data['timestamp'][i] > st:
            si = i
            break

    for i in range(len(d.data['timestamp'])):
        if d.data['timestamp'][i] > et:
            ei = i - 1
            break

    watt = 0
    x = d.data['timestamp']
    y = d.data['current_a'] * d.data['voltage_filtered_v']

    for i in range(si, ei):
        watt = watt + y[i]

    watt = watt / (ei - si)

    print('Average consumption = ', '{:.2f}'.format(watt), ' W')


def co_max_roll(ulog):

    """
    Max roll angle during climbing out evaluator
    """

    d = ulog.get_dataset('vehicle_attitude')
    t = ft_analyzer(ulog)
    pos_time = t[2]
    fwp_time = t[3]
    max_r = 0.0

    for i in range(len(d.data['timestamp'])):
        if d.data['timestamp'][i] > pos_time and d.data['timestamp'][i] < fwp_time:
            tmp_r = tl.degRoll(d.data['q[0]'][i], d.data['q[1]'][i], d.data['q[2]'][i], d.data['q[3]'][i])
            if abs(tmp_r) > abs(max_r):
                max_r = tmp_r

    return max_r


def rwto_max(ulog):

    """
    Max/Min angle and angular rate during rwto analyzer
    """

    d1 = ulog.get_dataset('vehicle_attitude')
    d2 = ulog.get_dataset('vehicle_angular_velocity')
    t = ft_analyzer(ulog)
    rwto_time = t[0]
    vr_time = t[1]
    max_ra = -100.0
    min_pa = 100.0
    max_pr = -100.0
    min_pr = 100.0

    for i in range(len(d1.data['timestamp'])):
        if d1.data['timestamp'][i] > rwto_time and d1.data['timestamp'][i] < vr_time:
            tmp_a = abs(tl.degRoll(d1.data['q[0]'][i], d1.data['q[1]'][i], d1.data['q[2]'][i], d1.data['q[3]'][i]))
            if tmp_a > max_ra:
                max_ra = tmp_a

    for i in range(len(d1.data['timestamp'])):
        if d1.data['timestamp'][i] > rwto_time and d1.data['timestamp'][i] < vr_time:
            tmp_a = tl.degPitch(d1.data['q[0]'][i], d1.data['q[1]'][i], d1.data['q[2]'][i], d1.data['q[3]'][i])
            if tmp_a < min_pa:
                min_pa = tmp_a

    for i in range(len(d2.data['timestamp'])):
        if d2.data['timestamp'][i] > rwto_time and d2.data['timestamp'][i] < vr_time:
            tmp_a = np.rad2deg(d2.data['xyz[1]'][i])
            if tmp_a > max_pr:
                max_pr = tmp_a
            elif tmp_a < min_pr:
                min_pr = tmp_a

    return max_ra, min_pa, max_pr, min_pr


def ft_analyzer(ulog):

    abr_off = int(0)
    abr_roll = int(0)
    abr_pt_ang = int(0)
    abr_pt_rate = int(0)

    rwto_time = 0.0
    vr_time = 0.0
    pos_time = 0.0
    fwp_time = 0.0
    slp_time = 0.0
    lnd_time = 0.0

    for m in ulog.logged_messages:
        mt = m.timestamp
        mmsg = m.message
        if 'Taking off.' in mmsg:
            rwto_time = mt
        if 'Takeoff airspeed reached' in mmsg:
            vr_time = mt
        if 'takeoff path' in mmsg:
            abr_off = abr_off + 1
        if 'roll angle' in mmsg:
            abr_roll = abr_roll + 1
        if 'pitch angle' in mmsg:
            abr_pt_ang = abr_pt_ang + 1
        if 'pitch angular rate' in mmsg:
            abr_pt_rate = abr_pt_rate + 1
        if 'Climbout' in mmsg:
            pos_time = mt
        if 'on slope' in mmsg:
            slp_time = mt
        if 'landed.' in mmsg:
            lnd_time = mt

    d = ulog.get_dataset('mission_result')

    for i in range(len(d.data['timestamp'])):
            if d.data['timestamp'][i] > vr_time:
                fwp_time = d.data['timestamp'][i] # first way point time
                break

    return rwto_time, vr_time, pos_time, fwp_time, slp_time, abr_off, abr_roll, abr_pt_ang, abr_pt_rate, lnd_time


def asp_set(ulog):

    d = ulog.initial_parameters
    asp_max = d['FW_AIRSPD_MAX']
    asp_trm = d['FW_AIRSPD_TRIM']
    asp_min = d['FW_AIRSPD_MIN']

    return asp_max, asp_trm, asp_min


def bat_analyzer(ulog):

    d1 = ulog.get_dataset('battery_status')
    d2 = ulog.get_dataset('vehicle_rates_setpoint')
    t = ft_analyzer(ulog)
    rwto_time = t[0]
    fwp_time = t[3]
    lnd_time = t[9]
    v_init = 0.0
    v_fin = 0.0
    v_drop = 100.0

    for i in range(len(d1.data['timestamp'])):
        if d1.data['timestamp'][i] > rwto_time:
            for j in range(len(d2.data['timestamp'])):
                if d2.data['timestamp'][j] > rwto_time:
                    if d2.data['thrust_body[0]'][j-1] < 0.01:
                        v_init = d1.data['voltage_filtered_v'][i-1]
                        break
                    else:
                        v_init = 999.9
                        break
            else:
                continue
            break

    for i in range(len(d1.data['timestamp'])):
        if d1.data['timestamp'][i] > lnd_time:
            for j in range(len(d2.data['timestamp'])):
                if d2.data['timestamp'][j] > lnd_time:
                    if d2.data['thrust_body[0]'][j] < 0.01:
                        v_fin = d1.data['voltage_filtered_v'][i]
                        break
                    else:
                        v_fin = 999.9
                        break
            else:
                continue
            break

    for i in range(len(d1.data['timestamp'])):
        if d1.data['timestamp'][i] > rwto_time and d1.data['timestamp'][i] < fwp_time:
            if v_drop > d1.data['voltage_filtered_v'][i]:
                v_drop = d1.data['voltage_filtered_v'][i]
        elif d1.data['timestamp'][i] > fwp_time:
            break

    v_drop = v_init - v_drop

    return v_init, v_fin, v_drop





#if __name__ == "__main__":
#    main()
