#!/usr/bin/env python
from utils import *
import subprocess

def device(dev):
    ret = subprocess.call("ls %s" % dev,
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0

def v4l_info(dev, search_string):
    ret = subprocess.call("v4l2-ctl --all -d %s | grep %s"%(dev, search_string) ,
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0

