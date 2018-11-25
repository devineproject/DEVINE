#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup()
setup_args['packages'] = ['devine_dialog']
setup_args['package_dir'] = {'': 'src'}

setup(**setup_args)
