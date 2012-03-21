#!/usr/bin/env python
from mitro_testing.utils import *
from mitro_testing import network, video, process, ros

def test_network():
    print '-'*80
    network_tests = []
    add_test(network_tests, network.ip, 'eth0', '10.10.14.1')
    add_test(network_tests, network.ping, 'localhost')
    add_test(network_tests, network.ping, 'bob')
    add_test(network_tests, network.ping, 'mitro-laptop')
    add_test(network_tests, network.ping, '10.10.10.1')
    add_test(network_tests, network.ping, 'www.google.com')
    group_test(network_tests)

def test_ros():
    print '-'*80
    ros_tests = []
    add_test(ros_tests, ros.roscore, )
    add_test(ros_tests, ros.rostopichz, '/base_scan', 10)
#    add_test(ros_tests, ros.rostopichz, '/cloud_obstacles', 10)
    group_test(ros_tests)

if __name__ == '__main__':

    test_network()
    test_ros()
