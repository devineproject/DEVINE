'''Image blur detection'''
import cv2
import numpy as np

DETECTION_THRESHOLD = 20

def is_image_blurry(img):
    """ compute the variance of the Laplacian to measure focus"""
    gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    focus_measure = cv2.Laplacian(gray, cv2.CV_64F).var() 
    return focus_measure < DETECTION_THRESHOLD
