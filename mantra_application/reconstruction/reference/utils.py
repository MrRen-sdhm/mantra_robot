import yaml
from yaml import CLoader
import os
import datetime


def get_curr_dir():
    return os.path.dirname(os.path.abspath(__file__))


def getDictFromYamlFilename(filename):
    """
    Read data from a YAML files
    """
    return yaml.load(file(filename), Loader=CLoader)


def saveToYaml(data, filename):
    with open(filename, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)


def get_current_YYYY_MM_DD_hh_mm_ss():
    """
    Returns a string identifying the current:
    - year, month, day, hour, minute, second

    Using this format:

    YYYY-MM-DD-hh-mm-ss

    For example:

    2018-04-07-19-02-50

    Note: this function will always return strings of the same length.

    :return: current time formatted as a string
    :rtype: string

    """

    now = datetime.datetime.now()
    string = "%0.4d-%0.2d-%0.2d-%0.2d-%0.2d-%0.2d" % (now.year, now.month, now.day, now.hour, now.minute, now.second)
    return string


def dictFromPosQuat(pos, quat):
    d = dict()
    d['translation'] = dict()
    d['translation']['x'] = pos[0]
    d['translation']['y'] = pos[1]
    d['translation']['z'] = pos[2]

    d['quaternion'] = dict()
    d['quaternion']['w'] = quat[0]
    d['quaternion']['x'] = quat[1]
    d['quaternion']['y'] = quat[2]
    d['quaternion']['z'] = quat[3]

    return d


def getQuaternionFromDict(d):
    quat = None
    quatNames = ['orientation', 'rotation', 'quaternion']
    for name in quatNames:
        if name in d:
            quat = d[name]

    if quat is None:
        raise ValueError("Error when trying to extract quaternion from dict, your dict doesn't contain a key in ['orientation', 'rotation', 'quaternion']")

    return quat
