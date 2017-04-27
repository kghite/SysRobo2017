#!/usr/bin/env python
"""Feature matcher. Subscribes to map, finds feature location in thresholded occupancy grid"""
"""Publishes marker to that location"""
import rospy
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import cPickle as pickle
import thread
from scipy import misc, special,stats
from matplotlib import pyplot as plt


class Feature_Matcher():
    def __init__(self):
        """ Initialize the robot control, """
        rospy.init_node('featurematcher')
        self.sleepy = rospy.Rate(2)
        #subscribe to map for occupancy grid, publish to elevator map
        rospy.Subscriber('/map', OccupancyGrid, self.process_occupancy_grid)
        self.map_pub = rospy.Publisher('/elevatormap', OccupancyGrid, queue_size = 10)
        self.last_map = None #save last map
        #path to feature to match in greyscale image form
        self.featurepath = "elevator_template.jpg"
        self.test_path = "test_image.jpg" #for test
        self.map_path = "map_image.jpg" #for test

        #load feature to compare to
        #self.features = np.load(self.featurepath)
        #image_cv2 = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        # load template sign images as grayscale
        self.elevators = cv2.imread(self.featurepath,0)
        #load map template as graysacle(for testing)
        # self.map_image = cv2.imread(self.test_path,0)

        self.test=False #whether or not to run the test picture printout

    def match_features(self):
        """ Matches features against template and publishes map with elevator location"""
        #load images into memory so if they're updated by a callback nothing gets broken

        #map_image = self.last_map

        if self.elevators is None:
            print 'Elevators are None. NOT MATCHING FEATURES. (Are you in the right subdirectory?)'
            return
        if self.last_map is None is None:
            print 'Elevators are good, last map is None. NOT MATCHING FEATURES'
            return

        print 'matching features'
        map_image = cv2.imread(self.map_path,0)
        elevator_image = self.elevators

        res = cv2.matchTemplate(map_image,elevator_image,eval('cv2.TM_CCOEFF'))

        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc #top left of where it's recognizing an elevator it's the "max location" coming out of the cv2 function
        confidence = max_val

        if self.test==True: #print picture things for debugging!
            w,h = elevator_image.shape[::-1]
            test_img = map_image.copy()
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(test_img,top_left, bottom_right, 255, 2)
            plt.subplot(121),plt.imshow(res,cmap = 'gray')
            plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
            plt.subplot(122),plt.imshow(test_img,cmap = 'gray')
            plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
            plt.suptitle('Testing!\n To zoom in, use Zoom To Rectangle Tool')
            plt.show()
        #create new blank array
        newgridarr = np.zeros(self.map_width*self.map_height)
        #make corresponding value 100% chance of occupancy
        newgridarr[top_left[0]*top_left[1]] = 100
        #make occupancy grid.
        newgrid = OccupancyGrid()
        newgrid.info.width = self.map_width
        newgrid.info.height = self.map_height
        newgrid.data = newgridarr

        #TODO: CONFIDENCE NUMBERS ARE WEIRD. Either scale them or adjust this threshhold
        if confidence > 60: #only publish if confidence is high
            self.map_pub.publish(newgrid)


    def process_occupancy_grid(self, occupancy_grid):
        '''
        Input: Occupancy grid
        Output: Binary openCV image of occupancy grid
        '''
        #read into numpy array and shape correctly
        np_arr = np.asarray(occupancy_grid.data)
        self.map_width = occupancy_grid.info.width
        self.map_height = occupancy_grid.info.height
        np_arr = np.reshape(np_arr,(occupancy_grid.info.width, occupancy_grid.info.height))
        #threshold to 0s and 1s
        binary_np = stats.threshold(stats.threshold(np_arr,0,101,0),0,1,255)
        self.last_map = cv2.imwrite(self.map_path,binary_np)
        print 'RECEIVED'
        #initiate feature matching. Whenever you receive a map, find elevators!
        self.match_features()

    def run(self):
        while not rospy.is_shutdown():
            #although it doesn't look like anything is running, trust us.
            #Feature matching is initiated by receiving a map from the /map rostopic.
            self.sleepy.sleep()

feature = Feature_Matcher()
feature.test=False #set me to true to get printing images for debugging!
feature.run()
