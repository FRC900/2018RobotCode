#!/usr/bin/env python
""" 
NikolausDemmel 
Github Gist
Created Feb 11, 2014
rosbag_filter_subsequence.sh
"""

import sys
import argparse
from fnmatch import fnmatchcase

import rospy

from rosbag import Bag

import yaml

def main():

    parser = argparse.ArgumentParser(description='Merge one or more bag files with the possibilities of filtering topics.')
    parser.add_argument('outputbag',
                        help='output bag file with topics merged')
    parser.add_argument('inputbag', nargs='+',
                        help='input bag files')
    parser.add_argument('-v', '--verbose', action="store_true", default=False,
                        help='verbose output')
    parser.add_argument('-t', '--topics', default="*",
                        help='string interpreted as a list of topics (wildcards \'*\' and \'?\' allowed) to include in the merged bag file')

    args = parser.parse_args()

    topics = args.topics.split(' ')

    #TODO: needs to reorganize bagfiles based on time started
    start_times = []
    for ifile in args.inputbag:
        info_dict = yaml.load(Bag(ifile, 'r')._get_yaml_info())
        start_times.append((info_dict["start"], ifile))
        start_times.sort()
    print(start_times)
     
    total_included_count = 0
    total_skipped_count = 0

    if (args.verbose):
        print("Writing bag file: " + args.outputbag)
        print("Matching topics against patters: '%s'" % ' '.join(topics))

    with Bag(args.outputbag, 'w') as o: 
        for start, ifile in start_times:
            matchedtopics = []
            included_count = 0
            skipped_count = 0
            if (args.verbose):
                print("> Reading bag file: " + ifile)
            with Bag(ifile, 'r') as ib: 
                for topic, msg, t in ib:
                    if any(fnmatchcase(topic, pattern) for pattern in topics):
                        if not topic in matchedtopics:
                            matchedtopics.append(topic)
                            if (args.verbose):
                                print("Including matched topic '%s'" % topic)
                        o.write(topic, msg, t)
                        included_count += 1
                    else:
                        skipped_count += 1
            total_included_count += included_count
            total_skipped_count += skipped_count
            if (args.verbose):
                print("< Included %d messages and skipped %d" % (included_count, skipped_count))

    if (args.verbose):
        print("Total: Included %d messages and skipped %d" % (total_included_count, total_skipped_count))

if __name__ == "__main__":
    main()
