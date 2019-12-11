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
  f_out = open(r'/home/mylabtop/bit_mbzirc/src/bit_hardware_interface/src/gps_kml.kml','w')
  lon = data.longitude
  lat = data.latitude
  alt = data.altitude
  print(lon,lat,alt)
  degree_lon = int(lon)
  minute_lon = int((lon - degree_lon)*1e2)
  second_lon = int((lon - degree_lon - minute_lon*0.01)*1e4)
  baifen_lon = (lon - degree_lon - minute_lon*0.01 - second_lon*0.0001)*1e6
  # print(degree_lon,minute_lon,second_lon,baifen_lon)
  degree_lat = int(lat)
  minute_lat = int((lat - degree_lat)*1e2)
  second_lat = int((lat - degree_lat - minute_lat*1e-2)*1e4)
  baifen_lat = (lat - degree_lat - minute_lat*1e-2 - second_lat*1e-4)*1e6
  # print(degree_lat,minute_lat,second_lat,baifen_lat)
  gpsData = str(degree_lon+minute_lon/60.0 + second_lon/60.0/60.0 + baifen_lon/100.0/60.0/60.0)+','+str(degree_lat+minute_lat/60.0 + second_lat/60.0/60.0 + baifen_lat/100.0/60.0/60.0)+' \n'  #','+str(alt)+
  # gpsData = str(lon)+','+str(lat)+' \n'
  datawriteto = datawriteto + gpsData
  writeto = msg + datawriteto + msg2
  f_out.write(writeto)
  f_out.close()


if __name__ == '__main__':
	rospy.init_node('gps_produce')
	rospy.Subscriber("/GPS/GPS_data", NavSatFix, callback)

	rospy.spin()
	

