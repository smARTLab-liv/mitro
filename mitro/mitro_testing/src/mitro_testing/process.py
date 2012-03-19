#!/usr/bin/env python
from utils import *
import subprocess

def running(name):
    ret = subprocess.call('ps ax | grep -v grep | grep "\<%s\>"' % name,
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0
    

