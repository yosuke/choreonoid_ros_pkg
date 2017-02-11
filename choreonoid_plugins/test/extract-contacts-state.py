#!/usr/bin/env python
#
#

from __future__ import print_function

import sys
import os
import argparse
import textwrap
import yaml

#
# Function.
#

def output_csv(data, seq):
    ret = 0

    if (('header' in data) and ('states' in data)):
        header = data['header']

        if (('seq' in header) and (header['seq'] == seq)):
            print('seq\n{0}'.format(header['seq']))

            states = data['states']

            for i in range(len(states)):
                state = states[i]
                #
                print('info\n{0}'.format(state['info']))
                print('total wrench, force x, force y, force z, torque x, torque y, torque z\n' +
                        ', {0}, {1}, {2}, {3}, {4}, {5}'.format(
                        state['total_wrench']['force']['x'], state['total_wrench']['force']['y'],
                        state['total_wrench']['force']['z'], state['total_wrench']['torque']['x'],
                        state['total_wrench']['torque']['y'], state['total_wrench']['torque']['z']))

                #
                wrenches  = state['wrenches']
                positions = state['contact_positions']
                normals   = state['contact_normals']

                print('wrenches positions normals depth, force x, force y, force z, torque x, torque y, torque z, ' +
                        ' position x, position y, position z, normal x, normal y, normal z, depth')

                for j in range(len(wrenches)):
                    print(', {0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}'.format(
                            wrenches[j]['force']['x'], wrenches[j]['force']['y'], wrenches[j]['force']['z'],
                            wrenches[j]['torque']['x'], wrenches[j]['torque']['y'], wrenches[j]['torque']['z'],
                            positions[j]['x'], positions[j]['y'], positions[j]['z'],
                            normals[j]['x'], normals[j]['y'], normals[j]['z'], state['depths'][j]))
            ret = 1
        if (('seq' in header) and (header['seq'] > seq)):
            ret = -1

    return ret

#
# Main.
#

if __name__ == '__main__':
    exitcode = 1
    parser = argparse.ArgumentParser(
                formatter_class=argparse.RawDescriptionHelpFormatter,
                description=textwrap.dedent('''\
                Do extract contacts state information of your choosen sequence No.

                 e.g.
                  $ rostopic echo <topic of contacts state> > contacts.log
                  $ %(prog)s -l contacts.log -s 2224

                If you use the gazebo_ros_bumper, run the following command.

                  $ cat contacts.log | sed -e's/info: Debug:/info:/' > contacts_fix.log
                  $ %(prog)s -l contacts_fix.log -s 2224

                This is because the invalid YAML format is included.
                '''))
    parser.add_argument('-l', '--log-file', type=str, required=True, help='specify by preserve the rostopic output')
    parser.add_argument('-s', '--sequence', type=int, required=True, help='specify any sequence No.')
    args   = parser.parse_args()

    try:
        exitcode = 0
        is_found = 0
        stream = file(args.log_file, 'r')

        for data in yaml.load_all(stream):
            is_found = output_csv(data, args.sequence)
            if (is_found): break
    except ValueError:
        pass

    if (is_found != 1):
        print('seq {0} not found'.format(args.sequence))

    sys.exit(exitcode)
