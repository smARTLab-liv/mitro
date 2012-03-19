#!/usr/bin/env python
from mitro_testing.utils import *
from mitro_testing import network, video

def test_network():
    network_tests = []
    add_test( network_tests , network.check_ip, 'eth0', '10.10.14.2' )
    add_test( network_tests , network.check_unconnected, 'wlan0' )
    add_test( network_tests , network.ping, 'localhost' )
    add_test( network_tests , network.ping, 'mitro-laptop' )
    add_test( network_tests , network.ping, 'bob' )
    add_test( network_tests , network.ping, 'www.google.com' )
    add_test( network_tests , network.ping, 'root' )
    group_test(network_tests)

def test_video():
    video_tests = []
    add_test( video_tests , video.check_device, '/dev/video1' )
    add_test( video_tests , video.check_device, '/dev/video2' )
    add_test( video_tests , video.check_device, '/dev/video3' )
    add_test( video_tests , video.check_v4l_info, '/dev/video1', 'uvcvideo' )
    add_test( video_tests , video.check_v4l_info, '/dev/video2', 'uvcvideo' )
    add_test( video_tests , video.check_v4l_info, '/dev/video3', 'Dummy' )
    group_test(video_tests)

if __name__ == '__main__':
    test_network()
    test_video()
