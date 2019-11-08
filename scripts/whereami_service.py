#!/usr/bin/env python3

from whereami import predict, predict_proba
import rospy
import json
import rospkg
import os
import sys
import yaml
import wifi_slam.srv
import wifi_slam.msg

rospack = rospkg.RosPack()
package_path = rospack.get_path('wifi_slam')
config_path = os.path.join(package_path, 'maps', 'whereami_locations.yml')
with open(config_path, 'r') as f:
    whereami_cfg = yaml.load(f)

def handle_get_location(req):

    loc = predict()
    loc_building = ''
    loc_floor = ''
    flag = 0

    buildings = list(whereami_cfg.keys())
    for building in buildings:
        floors = list(whereami_cfg[building].keys())
        for floor in floors:
            regions = whereami_cfg[building][floor]['regions']
            if loc in regions:
                loc_building = building
                loc_floor = floor

    return wifi_slam.srv.GetLocationResponse(loc_floor, loc_building, loc)

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
