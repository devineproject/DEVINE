#!/usr/bin/env python3
""" Wrapper to run the js-webpack watcher """
import os
from subprocess import call

CURRENT_DIR = os.path.dirname(os.path.realpath(__file__)) 
DASHBOARD_ROOT_DIR = os.path.join(CURRENT_DIR, '../../')

if __name__ == '__main__':
    call('npm run build-watch', cwd=DASHBOARD_ROOT_DIR, shell=True)
