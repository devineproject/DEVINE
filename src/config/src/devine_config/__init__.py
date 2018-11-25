__author__ = "Jordan Prince Tremblay, Ismael Balafrej, Felix Labelle, Félix Martel-Denis, Eric Matte, Adam Letourneau, Julien Chouinard-Beaupré, Antoine Mercier-Nicol"
__copyright__ = "Copyright 2018, DEVINE Project"
__credits__ = ["Simon Brodeur", "François Ferland", "Jean Rouat"]
__license__ = "BSD"
__version__ = "1.0.0"
__email__ = "devine.gegi-request@listes.usherbrooke.ca"
__status__ = "Production"

import os
import json

CONFIG_FILE_NAME = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'static', 'parameters.json')

with open(CONFIG_FILE_NAME) as f:
    config = json.load(f)


def topicname(metaname):
    """ Fetch the full name for a topic """
    return config['topics'][metaname]['name']


def constant(name):
    """ Fetch a constant by name """
    return config['constants'][name]


def printtopics():
    """ Write json topics to stdout """
    print(json.dumps(config['topics']))


def gettopics():
    """ Return the entire topics object """
    return config['topics']
