# -*- coding: utf-8 -*-
""" Image blur detection """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import cv2

DETECTION_THRESHOLD = 20
BLACK_THRESHOLD = 10


def is_image_blurry(img):
    """ Compute the variance of the Laplacian to measure focus """
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    focus_measure = cv2.Laplacian(gray, cv2.CV_64F).var()
    is_mostly_black_img = gray.mean() <= BLACK_THRESHOLD
    return not is_mostly_black_img and focus_measure < DETECTION_THRESHOLD
