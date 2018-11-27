import json
from devine_common import ros_utils


def load_dialogs():
    """ Load dialogs from json file """
    with open(ros_utils.get_fullpath(__file__, 'dialogs.json')) as file:
        return json.loads(file.read())