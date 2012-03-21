#!/usr/bin/env python
from utils import *
import subprocess
import time

def roscore():
    ret = subprocess.call("rostopic list",
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0
    
def rostopic(name):
    ret = subprocess.call("rostopic info %s"%name,
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0


def rostopichz(name, hz):
    if not test(rostopic, name):
        return False
    p = subprocess.Popen(["rostopic", "hz", name],
                          shell=False,
                          stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)
    time.sleep(2)
    p.terminate()
    out = p.communicate()[0]
    for l in out.split('\n'):
        if "average" in l:
            rate = float(l.split(" ")[-1])
            debug('%s published with rate: %dhz'%(name, rate))
            if rate >= hz:
                return True
    return False
