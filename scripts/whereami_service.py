#!/usr/bin/env python3

from whereami import predict, predict_proba
import rospy
import json
import rospkg
import os
import sys
import wifi_slam.srv
import wifi_slam.msg

def handle_get_location(req):

    loc = predict()

    return wifi_slam.srv.GetLocationResponse(loc)

def handle_get_all_prob(req):

    data_dict = predict_proba()

    return wifi_slam.srv.GetProbabilityResponse(json.dumps(data_dict))

def whereami_node():

    rospy.init_node('whereami_node')

    s1 = rospy.Service('get_location', wifi_slam.srv.GetLocation, handle_get_location)
    s2 = rospy.Service('get_all_prob', wifi_slam.srv.GetProbability, handle_get_all_prob)
#    s4 = rospy.Service('get_all_locations', GetAllLocations, handle_get_all_locations)

    rospy.spin()

if __name__=="__main__":

    whereami_node()
