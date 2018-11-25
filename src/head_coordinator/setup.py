#!/usr/bin/env python
# -*- coding: utf-8 -*-
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup()
setup_args['packages'] = ['devine_head_coordinator']
setup_args['package_dir'] = {'': 'include'}

setup(**setup_args)
