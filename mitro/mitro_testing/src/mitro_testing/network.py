#!/usr/bin/env python
from utils import *
import netifaces
import subprocess

def interface(name):
    return name in netifaces.interfaces()

def connected(name):
    if test(interface, name):
        return netifaces.AF_INET in netifaces.ifaddresses(name)
    else:
        return False

def unconnected(name):
    return not connected(name)
    
def ip(name, ip):
    if test(connected, name):
        return ip == netifaces.ifaddresses(name)[netifaces.AF_INET][0]['addr']
    else:
        return False

def ping(host):
    ret = subprocess.call("ping -c 1 %s" % host,
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0
    

