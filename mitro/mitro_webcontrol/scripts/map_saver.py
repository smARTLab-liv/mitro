#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import Image, ImageDraw

class MapSaver():

    file_name = '../src/static/images/map.png'
    image = None
    image_draw = None

    def __init__(self, *args, **kwargs):
        rospy.init_node('map_saver', anonymous=False)
        rospy.Subscriber("/map", OccupancyGrid, self.callback)
        rospy.spin()

    def callback(self, data):
        if not self.image:
            self.image = Image.new('RGB', (data.info.width, data.info.height))
            self.image_draw = ImageDraw.Draw(self.image)
        for y in range(0, data.info.height):
            for x in range(0, data.info.width):
                d = data.data[(data.info.width*y)+x]
                if d == -1:
                    color = (100,100,100)
                elif d == 0:
                    color = (255, 255, 255)
                else:
                    color = (0, 0, 0)
                self.image_draw.point((x, data.info.height - y), fill=color)

        self.image.save(self.file_name)
        #self.image.show()
        


if __name__ == '__main__':
    map_saver = MapSaver()
