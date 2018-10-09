#!/usr/bin/env python2
'''
    Zone detection using bright colored squares
    Created by DEVINE
    Based on: https://www.sciencedirect.com/science/article/pii/S0898122112002787
'''

from __future__ import division

import rospy
from std_msgs.msg import String

from ros_image_processor import ImageProcessor, ROSImageProcessingWrapper

import cv2
import numpy as np
from enum import Enum

# sudo pip install shapely
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from bson import json_util

from devine_config import topicname

IMAGE_TOPIC = topicname('zone_detection_image')
ZONE_DETECTION_TOPIC = topicname('zone_detection')

# Tracked colors
class TrackingColor(Enum):
    FUCHSIA = 1
    GREEN = 2

#Polygon data extracted from the COLOR_PICKER_MODE using parse_hls_dump.py

#Hue-saturation polygon of tracked colors
SATURATION_HUE_POLYGONS = {
    TrackingColor.FUCHSIA: Polygon([(162, 100), (175, 100), (178, 180), (178, 260), (162, 260)]),
    TrackingColor.GREEN: Polygon([(10, 100), (33, 100), (33, 150), (30, 200), (26, 254), (16, 254), (10, 225)])
}

#Hue-Lightness polygon of tracked colors
LIGHTNESS_HUE_POLYGONS = {
    TrackingColor.FUCHSIA: Polygon([(163, 100), (172, 100), (178, 152), (178, 161), (168, 200), (163, 180)]),
    TrackingColor.GREEN: Polygon([(15, 100), (32, 100), (32, 120), (29, 193), (10, 193), (10, 120)])
}

TOP_LEFT_CORNER = [TrackingColor.FUCHSIA, TrackingColor.GREEN]
BOTTOM_RIGHT_CORNER = [TrackingColor.GREEN, TrackingColor.FUCHSIA]

COLOR_DEBUG_MODE = False

# Collect new data 
COLOR_PICKER_MODE = False
HLSFILE = open('HLSDump.txt', 'a') if COLOR_PICKER_MODE else None
if COLOR_PICKER_MODE:
    np.set_printoptions(threshold=np.inf)

class ZoneDetection(ImageProcessor):
    '''Zone detection'''

    def detect_corner(self, corner_array, color_positions, image_width, image_height):
        '''Detect a corner based of the colors of the corner_array'''
        [color1, color2] = corner_array
        if color_positions.get(color1) and color_positions.get(color2):
            for (c1x, c1y) in color_positions[color1]:
                for (c2x, c2y) in color_positions[color2]:
                    if c1x < c2x and c1x + 0.2 * image_width > c2x: # validity in X
                        if abs(c1y - c2y) < 0.1 * image_height:     # validity in Y
                            return [c1x, (c1y + c2y) / 2]
        return [-1, -1]

    def filter_valid_shapes(self, image):
        '''Get bounding rects of quadrilateral shapes in the image'''
        _, contours, _ = cv2.findContours(
            image,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )

        def filter_contour(contour):
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            return len(approx) == 4

        filtered_contours = filter(filter_contour, contours)
        return map(cv2.boundingRect, filtered_contours)
    
    def process(self, image, _):
        '''Process a new image by detecting possible zones'''
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if COLOR_DEBUG_MODE or COLOR_PICKER_MODE else None
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        result = {}

        [width, height, _] = hls.shape

        if COLOR_PICKER_MODE:
            zone = cv2.selectROI(bgr)
            zone_color = hls[int(zone[1]):int(zone[1]+zone[3]), int(zone[0]):int(zone[0]+zone[2])]
            HLSFILE.write(str(zone_color))
        else:
            # Blur
            hls = cv2.blur(hls, (3, 3), (-1, -1))
            
            # Creation of the mask 
            masked_image = np.zeros((width, height, 1), dtype=np.uint8)

            # Resulting positions of the colors
            color_positions = {}

            for tracked_color in TrackingColor:
                saturation_polygon = SATURATION_HUE_POLYGONS[tracked_color]
                lightness_polygon = LIGHTNESS_HUE_POLYGONS[tracked_color]
                for x in range(width):
                    for y in range(height):
                        [h, l, s] = hls[x, y]
                        if saturation_polygon.contains(Point(h, s)) and lightness_polygon.contains(Point(h, l)):
                            masked_image[x, y] = 255 #(255, 255, 255)

                # Noise reduction equivalent to erosion + dilatation filters
                masked_image = cv2.morphologyEx(masked_image, cv2.MORPH_OPEN, None, iterations=2)

                bounding_rects = self.filter_valid_shapes(masked_image)

                positions_detected = []
                for (x, y, w, h) in bounding_rects:
                    if w > 10 and h > 10:
                        positions_detected.append((x + w/2, y + h/2))
                        if COLOR_DEBUG_MODE:
                            cv2.rectangle(bgr, (x, y), (x+w, y+h), (255, 0, 0), 2)
                        
                color_positions[tracked_color] = positions_detected
            
                if COLOR_DEBUG_MODE:
                    cv2.imshow("Mask", masked_image)
                    cv2.imshow("Image", bgr)
                    cv2.waitKey(1)
                    break #Only the first color in debug mode, so we can show the results properly

            result = {
                "top_left_corner": self.detect_corner(TOP_LEFT_CORNER, color_positions, width, height),
                "bottom_right_corner": self.detect_corner(BOTTOM_RIGHT_CORNER, color_positions, width, height)
            }

        return json_util.dumps(result)


def main():
    '''Entry point of this file'''
    processor = ROSImageProcessingWrapper(ZoneDetection, IMAGE_TOPIC)
    publisher = rospy.Publisher(ZONE_DETECTION_TOPIC, String, queue_size=10, latch=True)
    processor.loop(lambda processor_output : publisher.publish(processor_output))

if __name__ == '__main__':
    main()
