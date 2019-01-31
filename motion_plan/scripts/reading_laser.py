#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
  '''
  The above code converts the 720 readings contained 
  inside the LaserScan msg into five distinct readings. 
  Each reading is the minimum distance measured on a sector 
  of 60 degrees (total 5 sectors = 180 degrees).
  '''
  # 720/5 = 144
  regions = [ 
  min(msg.ranges[0:143]),
  min(msg.ranges[144:287]),
  min(msg.ranges[288:431]),
  min(msg.ranges[432:575]),
  min(msg.ranges[576:713]),
  ]
  rospy.loginfo(regions)

def main():
  rospy.init_node('reading_laser')
  sub = rospy.Subscriber("/m2wr/laser/scan", LaserScan, clbk_laser)

  rospy.spin()

if __name__ == '__main__':
  main()