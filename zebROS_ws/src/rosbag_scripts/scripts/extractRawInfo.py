#!/usr/bin/python

# Copyright 2010 Ankur Sinha 
# Author: Ankur Sinha <sanjay DOT ankur AT gmail DOT com> 
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# File : extractRawInfo.py
#

import rosbag
import sys
import os
import json
import pickle
from rospy_message_converter import json_message_converter

# Global variable for input file name

def run():
    """
    Main run method. Calls other helper methods to get work done
    """

    if len(sys.argv) != 2:
        sys.stderr.write('[ERROR] This script only takes input bag file as argument.\n')
    else:
        inputFileName = sys.argv[1]
        print "[OK] Found bag: %s" % inputFileName

        bag = rosbag.Bag(inputFileName)
        topicList = readBagTopicList(bag)

        while True:
            if len(topicList) == 0:
                print "No topics in list. Exiting"
                break
            selection  = menu(topicList)

            if selection == -92:
                print "[OK] Printing them all"
                for topic in topicList:
                    extract_data(bag, topic, inputFileName)
                break
            elif selection == -45:
                break
            else:
                topic = topicList[selection]
                extract_data(bag, topic, inputFileName)
                topicList.remove(topicList[selection])

        bag.close()

def extract_data (bag, topic, inputFileName):
    """ 
    Spew messages to a file

    args: 
        topic -> topic to extract and print to txt file
    """

    outputFileName = os.path.splitext(os.path.split(inputFileName)[1])[0] + topic.replace("/","-") + ".txt"
    f = open(outputFileName, 'w')
    print "[OK] Printing %s" % topic
    print "[OK] Output file will be called %s." % outputFileName

    topic_all_msgs = {}

    outputFh = open(outputFileName, "w")
    
    for index, (topic, msg, t) in enumerate(bag.read_messages(topics=topic)):
        topic_all_msgs[index] = json.loads(json_message_converter.convert_ros_message_to_json(msg))

    json.dump(topic_all_msgs, outputFh)
    outputFh.close()
    print "[OK] DONE"

def menu (topicList):
    """
    Print the user menu and take input

    args:
        topicList: tuple containing list of topics

    returns:
        selection: user selection as integer
    """

    i = 0
    for topic in topicList:
        print '[{0}] {1}'.format(i, topic)
        i = i+1
    if len(topicList) > 1:
        print '[{0}] Extract all'.format(len(topicList))
        print '[{0}] Exit'.format(len(topicList) + 1)
    else:
        print '[{0}] Exit'.format(len(topicList))

    
    while True:
        print 'Enter a topic number to extract raw data from:'
        selection = raw_input('>>>')
        if int(selection) == len(topicList):
            return -92 # print all
        elif int(selection) == (len(topicList) +1):
            return -45 # exit
        elif (int(selection) < len(topicList)) and (int(selection) >= 0):
            return int(selection)
        else:
            print "[ERROR] Invalid input"


def readBagTopicList(bag):
    """
    Read and save the initial topic list from bag
    """
    print "[OK] Reading topics in this bag. Can take a while.."
    topicList = []
    for topic, msg, t in bag.read_messages():
        if topicList.count(topic) == 0:
            topicList.append (topic)

    print '{0} topics found:'.format(len(topicList))
    return topicList

if __name__ == "__main__":
    run()

