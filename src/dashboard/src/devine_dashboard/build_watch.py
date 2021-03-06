#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" Wrapper to run the js-webpack watcher """
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Felix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupre, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "Francois Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import os
from subprocess import call

CURRENT_DIR = os.path.dirname(os.path.realpath(__file__)) 
DASHBOARD_ROOT_DIR = os.path.join(CURRENT_DIR, '../../')

if __name__ == '__main__':
    call('npm run build-watch', cwd=DASHBOARD_ROOT_DIR, shell=True)
