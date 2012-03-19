#!/usr/bin/env python
from utils import *
import subprocess

def roscore():
    ret = subprocess.call("rostopic list",
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0
    

