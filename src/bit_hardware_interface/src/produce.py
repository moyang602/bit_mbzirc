#!/usr/bin/env python
#coding=utf-8

import roslib
import rospy

from sensor_msgs.msg import NavSatFix

import time

msg = '''<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<kml xmlns="http://earth.google.com/kml/2.2">
  <Document>
    <name>X-GPS Explorer</name>
    <Style id="X-GPSExplorer">
      <IconStyle>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>
        </Icon>
        <hotSpot x="32" y="1" xunits="pixels" yunits="pixels" />
      </IconStyle>
    </Style>
    <Folder>
      <name>Place Mark</name>
    </Folder>
    <Placemark>
      <name>My Path</name>
      <Style>
        <LineStyle>
          <color>ff0000ff</color>
          <width>2</width>
      </LineStyle>
      </Style>
      <LineString>
        <coordinates>
'''

msg2 = '''
		</coordinates>
			</LineString>
			</Placemark>
		</Document>
		</kml>
'''
datawriteto = '''

'''
def callback(data):
	global datawriteto
	f_out = open(r'/home/ugvcontrol/bit_mbzirc/src/bit_hardware_interface/src/gps_kml.kml','w')
	lon = data.longitude
	lat = data.latitude
	alt = data.altitude
	print(lon,lat,alt)
	gpsData = str(lon)+','+str(lat)+','+str(alt)+' \n'
	datawriteto = datawriteto + gpsData
	writeto = msg + datawriteto + msg2
	f_out.write(writeto)
	f_out.close()


if __name__ == '__main__':
	rospy.init_node('gps_produce')
	rospy.Subscriber("gps_data", NavSatFix, callback)

	rospy.spin()
	

