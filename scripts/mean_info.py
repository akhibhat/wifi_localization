#!/usr/bin/env python3

import rospy
import rospkg
import os
import sys
import math
import subprocess
from std_msgs.msg import Bool
import wifi_slam.msg
from access_points import IwlistWifiScanner as iws

class InfoTransmission():
    def __init__(self):
        rospy.init_node("information", anonymous=True)
        # Load the list of access points into a variable
        self.accessPoints = []
        self.filename = "Levine4.txt"
        self.get_apList()
        self.interface = "wlp3s0"
        self.wifi_interface = iws(self.interface)

        self.info_pub = rospy.Publisher("wifi_info", wifi_slam.msg.WifiInfo, queue_size = 1)
        rospy.Subscriber("reached_goal", Bool, self.reachGoalCallback)

        rospy.spin()

    def reachGoalCallback(self, msg):

        information = WifiInfo()
        information.wifiMean = []
        information.wifiStd = []

        count = 0
        if msg.data == 1:

            # DO 10 times:
            # Run wifi commands which will give you a list of accesspoint class objects.
            # For ap in accesspoints - check if the mac address is one of the reference
            # If it is, add the strength value into the vector, also add squares
            #
            # Outside the while loop, calculate the mean and standard deviation
            # Publish the stuff on the topic
            # Publish that you are done collecting information to the topic
            information.wifiMean, information.wifiStd = self.get_wifi_data()

            information.wifiMean = information.wifiMean/10
            information.wifiStd = math.sqrt(self.information.wifiStd/10)
            self.info_pub.publish(information)

    def get_apList(self):
        ap_file = open(self.filename, 'r')
        ap_list = ap_file.readlines()

        for ap in ap_list:
            self.accessPoints = ap[0:-1]

        self.num_ap = len(self.accessPoints)

    def get_wifi_data():
        updated_aplist = []

        means = zeros(self.num_ap, 1)
        stds = zeros(self.num_ap, 1)

        while(count < 10):
            accesspoints = self.wifi_interface.get_access_points()

            for ap in accesspoints:
                bssid = ap.bssid
                if bssid in self.accessPoints:
                    updated_aplist.append(ap)

            for ap in updated_aplist:
                idx = self.accessPoints.index(ap)
                means[idx] = means[idx] + ap.quality
                stds[idx] = stds[idx] + math.pow(ap.quality,2)

        return means, stds

if __name__== "__main__":
    filename = "Levine4.txt"
    Info = InfoTransmission()
