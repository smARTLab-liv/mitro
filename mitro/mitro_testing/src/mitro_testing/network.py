#!/usr/bin/env python
from utils import *
import netifaces
import subprocess

def check_interface(name):
    return name in netifaces.interfaces()

def check_connected(name):
    if test(check_interface, name):
        return netifaces.AF_INET in netifaces.ifaddresses(name)
    else:
        return False

def check_unconnected(name):
    return not check_connected(name)
    
def check_ip(name, ip):
    if test(check_connected, name):
        return ip == netifaces.ifaddresses(name)[netifaces.AF_INET][0]['addr']
    else:
        return False

def ping(host):
    ret = subprocess.call("ping -c 1 %s" % host,
                          shell=True,
                          stdout=open('/dev/null', 'w'),
                          stderr=subprocess.STDOUT)
    return ret == 0
    

