#!/usr/bin/env python
"""Helper script that makes published map a croppable jpg"""
import rospy
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import cPickle as pickle
import thread
from scipy import misc, special,stats



class Map_To_Image():

    def __init__(self):
        """ Initialize the robot control, """
        rospy.init_node('Maptoimage')
        self.sleepy = rospy.Rate(2)
        #subscribe to map for occupancy grid
        rospy.Subscriber('/map',OccupancyGrid, self.process_occupancy_grid)
        #save last map
        self.last_map = None
        #path to feature to match in greyscale image form
        self.featurepath = "elevator_template.jpg"
        #load feature to compare to 
        #self.features = np.load(self.featurepath)
        #image_cv2 = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        
 
    def match_features(self):
        """ Matches features against template and publishes map with elevator location"""
        #load images into memory so if they're updated by a callback nothing gets broken
        pause = raw_input("Press Enter to Make JPG")
        map_image = self.last_map
        print map_image
        cv2.imshow('map',map_image)
        cv2.imwrite( "Gray_Image.jpg", map_image);




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

feature = Map_To_Image()
feature.run()
