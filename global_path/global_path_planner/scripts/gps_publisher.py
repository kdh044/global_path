#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
import json

if __name__ == "__main__":
    rospy.init_node("fake_gps_publisher")
    
    # 기존 gps_data 토픽 (웹용)
    gps_data_pub = rospy.Publisher("gps_data", String, queue_size=10)
    
    # 새로 추가: ublox/fix 토픽 (path_visualizer.py용)
    ublox_fix_pub = rospy.Publisher("/ublox/fix", NavSatFix, queue_size=10)
    
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        # 기존 가짜 GPS 데이터
        fake_data = {
            "latitude": 35.8450893,
            "longitude": 127.13304149999999 
        }
        
        # 1. 기존 gps_data 토픽 발행 (웹용)
        gps_data_pub.publish(json.dumps(fake_data))
        
        # 2. 새로 추가: ublox/fix 토픽 발행 (path_visualizer.py용)
        fix_msg = NavSatFix()
        fix_msg.header.stamp = rospy.Time.now()
        fix_msg.header.frame_id = "gps_link"
        fix_msg.status.status = NavSatStatus.STATUS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = fake_data["latitude"]
        fix_msg.longitude = fake_data["longitude"]
        fix_msg.altitude = 100.0
        fix_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        ublox_fix_pub.publish(fix_msg)
        
        rospy.loginfo(f"Send GPS: {fake_data} (gps_data + ublox/fix)")
        rate.sleep()