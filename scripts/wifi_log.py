#!/usr/bin/env python3

import os
import sys
import yaml
import subprocess
import rospy
import signal
from std_msgs.msg import String
from std_msgs.msg import Int64
from access_points import IwlistWifiScanner as iws

class Wifi_Scanner():

    def __init__(self):
        rospy.init_node("wifi_scanner", anonymous=True)
        self.access_points = []
        self.filename = "myfile.txt"
        self.interface = 'wlp3s0'
        self.current_wifi_pub = rospy.Publisher('/current_wifi', String, queue_size=10)
        self.wifi_ap_pub = rospy.Publisher('/wifi_access_points', String, queue_size=10)
        rospy.Timer(rospy.Duration(5), self.local_goal_callback)
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.spin()

    def get_wifi_data(self):
        wifi_interface = iws(self.interface)
        wifi_access_points = wifi_interface.get_access_points()

        return wifi_access_points

    def get_current_wifi(self):
        strength_cmd = "iwconfig {} | grep Link".format(self.interface)
        mac_cmd = "iwconfig {} | grep Access".format(self.interface)

        strength = subprocess.Popen(strength_cmd, stdout=subprocess.PIPE, shell=True)
        address = subprocess.Popen(mac_cmd, stdout=subprocess.PIPE, shell=True)

        [str_out, str_err] = strength.communicate()
        [add_out, add_err] = address.communicate()

#        msg = out.decode('utf-8').strip()
        current_wifi_strength = str_out.decode('utf-8').strip()[-7:-4];
        current_mac_address = add_out.decode('utf-8').strip()[-17:]

        return current_wifi_strength, current_mac_address

    def local_goal_callback(self, event):

        wifi_strength, mac_address = self.get_current_wifi()
        current_wifi = String()
        current_wifi.data = wifi_strength + " " + mac_address

        self.current_wifi_pub.publish(current_wifi)

        self.update_aps()

        print(len(self.access_points))

    def update_aps(self):

        current_ap = self.get_wifi_data()

        for ap in current_ap:
            count = 0
            for access_point in self.access_points:
                if ap.bssid == access_point.bssid:
                    count = count + 1
            if count == 0:
                self.access_points.append(ap)

    def signal_handler(self,sig, frame):
        file1 = open(self.filename,'w')
        for access_pt in self.access_points:
            if access_pt.ssid == "AirPennNet-Device":
                file1.write(access_pt.bssid + "\n")
        file1.close()
        sys.exit(0)

if __name__=="__main__":
    interface = 'wlp3s0'
    WS = Wifi_Scanner()

