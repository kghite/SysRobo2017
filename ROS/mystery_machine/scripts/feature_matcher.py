#!/usr/bin/env python
"""Feature matcher. Subscribbes to map, finds feature location in thresholded occupancy grid"""
"""Publishes marker to that location"""
import rospy
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import cPickle as pickle
import thread
from scipy import misc, special,stats



class Feature_Matcher():

    def __init__(self):
        """ Initialize the robot control, """
        rospy.init_node('featurematcher')
        self.sleepy = rospy.Rate(2)
        #subscribe to map for occupancy grid
        rospy.Subscriber('/map',OccupancyGrid, self.process_occupancy_grid)
        #save last map
        self.last_map = None
        #path to feature to match in greyscale image form
        self.featurepath = "elevator_template.jpg"
        self.testpath = "test_image.jpg" #for test
        #load feature to compare to 
        #self.features = np.load(self.featurepath)
        #image_cv2 = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        
        # load template sign images as grayscale
        self.elevators = cv2.imread(self.featurepath,0)
        #load map template as graysacle(for testing)
        self.mapimage = cv2.imread(self.testpath,0)
        
        map_pub = rospy.Publisher('/elevatormap',OccupancyGrid, queue_size = 10)
 
    def match_features(self):
        """ Matches features against template and publishes map with elevator location"""
        #load images into memory so if they're updated by a callback nothing gets broken
        
        #map_image = self.last_map

        map_image = self.mapimage
        elevator_image = self.elevators
        #
        res =cv2.matchTemplate(map_image,elevator_image,'cv2.TM_CCOEFF')
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        confidence = max_val
        #create new blank array
        newgridarr = np.zeroes(self.map_width*self.map_height)
        #make corresponding value 100% chance of occupancy
        newgridarr[max_loc[0]*top_left[1]] = 100
        #make occupancy grid.
        newgrid = OccupancyGrid()
        newgrid.info.widgh = self.map_width
        newgrid.info.height = self.map_height
        newgrid.data = newgridarr

        #only publish if confidence is high
        if confidence > 60:
            map_pub.publish(newgrid)



    def process_occupancy_grid(self, occupancy_grid):
        '''
        Input: Occuapancy grid 
        Output: Binary openCV image of occupancy grid
        '''
        #read into numpy array and shape correctly
        np_arr = np.asarray(occupancy_grid.data)
        self.map_width = occupancy_grid.info.width
        self.map_height = occupancy_grid.info.height
        np_arr = np.reshape(np_arr,(occupancy_grid.info.width, occupancy_grid.info.height))
        #threshold to 0s and 1s
        binary_np = stats.threshold(stats.threshold(np_arr,0,101,0),0,1,255)
        self.last_map = binary_np


    ##Main

    def run(self):    
        self.sleepy.sleep()
        self.match_features()

feature = Feature_Matcher()
feature.run()
