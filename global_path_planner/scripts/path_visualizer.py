#!/usr/bin/env python3

import rospy
import rosbag
import json
import subprocess
import threading
import time
import utm
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import NavSatFix
import tf2_ros
from geometry_msgs.msg import TransformStamped

class SimplifiedPathVisualizer:
    def __init__(self):
        rospy.init_node('simplified_path_visualizer', anonymous=True)
        
        # Configuration
        self.bag_file = "/media/danny/DB/JBNU_LIDAR_DATASET_EVAL/2025-01-11-10-57-18.bag"
        
        # GPS origin from bag file
        self.gps_origin = None
        self.utm_origin = None
        
        # Data storage
        self.fasterlio_trajectory = []  
        self.latest_waypoints = None   
        self.gps_trajectory = []
        
        # 5M 주행 관리
        self.stage = "WAITING"  # WAITING -> MOVING_5M -> HEADING_SET
        self.initial_pose = None
        self.heading_offset = None
        
        # Publishers
        self.path_marker_pub = rospy.Publisher("/fasterlio_path", Marker, queue_size=10)
        self.waypoints_marker_pub = rospy.Publisher("/global_waypoints", MarkerArray, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/gps_path", Marker, queue_size=10)
        self.gps_pub = rospy.Publisher("/gps_data", String, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber("/Odometry", Odometry, self.fasterlio_callback)
        rospy.Subscriber("/waypoints", String, self.waypoints_callback)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gps_callback)  # GPS 실시간 수신
        
        # Timers
        rospy.Timer(rospy.Duration(0.5), self.publish_markers)
        rospy.Timer(rospy.Duration(1.0), self.handle_5m_navigation)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        rospy.Timer(rospy.Duration(1.0), self.publish_gps_data)
        
        rospy.loginfo("🚀 간단한 경로 시각화 시작!")
        
        # GPS 원점만 먼저 추출 (빠르게)
        self.extract_gps_origin()
        
        # bag 파일 재생 시작
        threading.Timer(3.0, self.start_bag_playback).start()
    
    def extract_gps_origin(self):
        """Bag 파일에서 GPS 원점만 빠르게 추출"""
        rospy.loginfo("📍 GPS 원점 추출 중...")
        try:
            with rosbag.Bag(self.bag_file, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=['/ublox/fix']):
                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                        if msg.status.status >= 0:
                            self.gps_origin = {"lat": msg.latitude, "lon": msg.longitude}
                            easting, northing, zone_num, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
                            self.utm_origin = {"easting": easting, "northing": northing, "zone": f"{zone_num}{zone_letter}"}
                            rospy.loginfo(f"🎯 GPS 원점: {self.gps_origin}")
                            break
        except Exception as e:
            rospy.logerr(f"❌ GPS 원점 추출 실패: {e}")
            self.gps_origin = {"lat": 35.8450893, "lon": 127.13304149999999}
    
    def gps_distance_check(self, new_gps):
        """GPS 거리 체크 (1미터 이상 이동시만 추가)"""
        if not self.gps_trajectory:
            return True
        
        last_gps = self.gps_trajectory[-1]
        last_x, last_y = self.gps_to_local_utm(last_gps["lat"], last_gps["lon"])
        new_x, new_y = self.gps_to_local_utm(new_gps["lat"], new_gps["lon"])
        
        distance = math.sqrt((new_x - last_x)**2 + (new_y - last_y)**2)
        return distance > 1.0
    
    def fasterlio_distance_check(self, new_point):
        """FasterLIO 거리 체크 (1미터 이상 이동시만 추가)"""
        if not self.fasterlio_trajectory:
            return True
        
        last_point = self.fasterlio_trajectory[-1]
        distance = math.sqrt((new_point["x"] - last_point["x"])**2 + (new_point["y"] - last_point["y"])**2)
        return distance > 1.0
    
    def gps_to_local_utm(self, lat, lon):
        """GPS를 로컬 UTM으로 변환"""
        if not self.utm_origin:
            return 0.0, 0.0
        easting, northing, _, _ = utm.from_latlon(lat, lon)
        x = easting - self.utm_origin["easting"]
        y = northing - self.utm_origin["northing"]
        return x, y
    
    def start_bag_playback(self):
        """Bag 파일 재생 (로그 숨기기)"""
        rospy.loginfo("🎬 Bag 파일 재생 시작...")
        bag_cmd = f"rosbag play --clock --rate=1.0 --quiet {self.bag_file}"  # --quiet 추가
        subprocess.Popen(bag_cmd, shell=True)
    
    def fasterlio_callback(self, msg):
        """FasterLIO 경로 실시간 생성 (heading 보정 적용)"""
        pos = msg.pose.pose.position
        pt = {"x": pos.x, "y": pos.y, "z": pos.z, "timestamp": msg.header.stamp.to_sec()}
        
        # GPS 원점에 맞춰 원점 보정
        if self.gps_origin and len(self.fasterlio_trajectory) == 0:
            # 첫 번째 FasterLIO 포인트를 GPS 원점으로 이동
            gps_origin_utm_x, gps_origin_utm_y = self.gps_to_local_utm(self.gps_origin["lat"], self.gps_origin["lon"])
            self.fasterlio_origin = {"x": pt["x"], "y": pt["y"]}  # 원본 저장
            pt["x"] = gps_origin_utm_x
            pt["y"] = gps_origin_utm_y
        elif hasattr(self, 'fasterlio_origin'):
            # 원점 보정 적용
            gps_origin_utm_x, gps_origin_utm_y = self.gps_to_local_utm(self.gps_origin["lat"], self.gps_origin["lon"])
            pt["x"] = pt["x"] - self.fasterlio_origin["x"] + gps_origin_utm_x
            pt["y"] = pt["y"] - self.fasterlio_origin["y"] + gps_origin_utm_y
        
        # ✅ 새로 추가: heading 보정이 완료되었다면 실시간 적용
        if self.heading_offset is not None:
            pt = self.apply_heading_correction(pt)
        
        # 1미터 간격으로 필터링
        if not self.fasterlio_trajectory or self.fasterlio_distance_check(pt):
            self.fasterlio_trajectory.append(pt)
            rospy.loginfo_throttle(3, f"🟢 FasterLIO: {len(self.fasterlio_trajectory)}개 포인트")
    
    def gps_callback(self, msg):
        """GPS 경로 실시간 생성"""
        if msg.status.status >= 0:
            gps_point = {"lat": msg.latitude, "lon": msg.longitude, "timestamp": msg.header.stamp.to_sec()}
            
            # 1미터 간격으로 필터링
            if not self.gps_trajectory or self.gps_distance_check(gps_point):
                self.gps_trajectory.append(gps_point)
                rospy.loginfo_throttle(3, f"📡 GPS: {len(self.gps_trajectory)}개 포인트")
    
    def waypoints_callback(self, msg):
        """웨이포인트 저장"""
        try:
            waypoints_data = json.loads(msg.data)
            if "waypoints" in waypoints_data:
                self.latest_waypoints = waypoints_data["waypoints"]
                rospy.loginfo(f"🗺️ 웨이포인트 수신: {len(self.latest_waypoints)}개")
        except Exception as e:
            rospy.logerr(f"❌ Waypoints 오류: {e}")
    
    def handle_5m_navigation(self, event):
        """5M 주행 → Heading 계산"""
        if self.stage == "WAITING" and len(self.fasterlio_trajectory) > 5:  # 5개만 있어도 시작
            self.start_5m_analysis()
        elif self.stage == "MOVING_5M":
            self.calculate_heading_offset()
    
    def start_5m_analysis(self):
        """5M 구간 분석 시작 (전체 경로에서 찾기)"""
        if len(self.fasterlio_trajectory) < 2:
            return
            
        # 첫 번째 포인트를 시작점으로 설정
        start_point = self.fasterlio_trajectory[0]
        
        # 전체 경로에서 5M 이동 구간 찾기 (10개 제한 제거)
        for i, point in enumerate(self.fasterlio_trajectory):
            distance = math.sqrt(
                (point["x"] - start_point["x"])**2 + 
                (point["y"] - start_point["y"])**2
            )
            if distance >= 5.0:
                self.initial_pose = start_point
                self.end_pose = point
                self.stage = "MOVING_5M"
                rospy.loginfo(f"🎯 5M 구간 발견: {distance:.2f}m (포인트 0 → {i})")
                return
        
        # 5M 구간을 아직 못 찾았다면
        current_max = 0
        if len(self.fasterlio_trajectory) > 1:
            current_max = math.sqrt(
                (self.fasterlio_trajectory[-1]["x"] - start_point["x"])**2 + 
                (self.fasterlio_trajectory[-1]["y"] - start_point["y"])**2
            )
        rospy.loginfo_throttle(3, f"🚶 5M 구간 대기 중... 현재 최대: {current_max:.2f}m")
    
    def calculate_heading_offset(self):
        """GPS와 FasterLIO 비교해서 heading 오프셋 계산"""
        if not hasattr(self, 'end_pose') or not self.initial_pose:
            return
        
        # FasterLIO 5m 이동 벡터
        dx = self.end_pose["x"] - self.initial_pose["x"]
        dy = self.end_pose["y"] - self.initial_pose["y"]
        fasterlio_heading = math.atan2(dy, dx)
        
        # 같은 시간대 GPS 데이터 찾기
        start_time = self.initial_pose["timestamp"]
        end_time = self.end_pose["timestamp"]
        
        gps_start = self.find_gps_by_time(start_time)
        gps_end = self.find_gps_by_time(end_time)
        
        if not gps_start or not gps_end:
            rospy.logwarn("⚠️ 해당 시간의 GPS 데이터 없음")
            return
        
        # GPS UTM 좌표로 변환해서 방향 계산 (더 정확)
        gps_start_utm_x, gps_start_utm_y = self.gps_to_local_utm(gps_start["lat"], gps_start["lon"])
        gps_end_utm_x, gps_end_utm_y = self.gps_to_local_utm(gps_end["lat"], gps_end["lon"])
        
        gps_dx = gps_end_utm_x - gps_start_utm_x
        gps_dy = gps_end_utm_y - gps_start_utm_y
        gps_heading = math.atan2(gps_dy, gps_dx)
        
        # 오프셋 계산
        self.heading_offset = gps_heading - fasterlio_heading
        self.stage = "HEADING_SET"
        
        rospy.loginfo(f"🧭 Heading 보정 완료!")
        rospy.loginfo(f"📍 FasterLIO 방향: {math.degrees(fasterlio_heading):.1f}도")
        rospy.loginfo(f"📍 GPS 실제 방향: {math.degrees(gps_heading):.1f}도") 
        rospy.loginfo(f"📍 보정 오프셋: {math.degrees(self.heading_offset):.1f}도")
        
        # 즉시 경로 업데이트
        self.update_corrected_trajectory()
    
    def update_corrected_trajectory(self):
        """heading 보정 적용한 새 경로 생성 (전체 경로를 원점 기준 회전)"""
        if self.heading_offset is None:
            return
            
        rospy.loginfo("🔄 FasterLIO 전체 경로를 원점 기준으로 회전 중...")
        
        # 모든 FasterLIO 포인트에 원점 기준 회전 적용
        rotated_trajectory = []
        for point in self.fasterlio_trajectory:
            rotated_point = self.apply_heading_correction(point)
            rotated_trajectory.append(rotated_point)
        
        # 회전된 경로로 교체
        self.fasterlio_trajectory = rotated_trajectory
        
        rospy.loginfo("✅ 전체 경로 회전 완료! 초록선이 시계처럼 회전했어야 합니다.")
    
    def find_gps_by_time(self, target_time):
        """특정 시간에 가장 가까운 GPS 데이터 찾기"""
        if not self.gps_trajectory:
            return None
            
        closest_gps = None
        min_time_diff = float('inf')
        
        for gps_point in self.gps_trajectory:
            time_diff = abs(gps_point["timestamp"] - target_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_gps = gps_point
        
        return closest_gps
    
    def calculate_gps_bearing(self, gps_start, gps_end):
        """GPS 두 점 사이의 방향각 계산"""
        lat1, lat2 = math.radians(gps_start["lat"]), math.radians(gps_end["lat"])
        lon1, lon2 = math.radians(gps_start["lon"]), math.radians(gps_end["lon"])
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return math.atan2(y, x)
    
    def apply_heading_correction(self, point):
        """FasterLIO 좌표에 heading 보정 적용 (GPS 원점 기준 회전)"""
        if self.heading_offset is None:
            return point
        
        # GPS 원점(0,0) 기준 상대좌표 (전체 경로를 원점 중심으로 회전)
        rel_x = point["x"] - 0  # GPS 원점
        rel_y = point["y"] - 0  # GPS 원점
        
        # 회전 적용 (시계처럼 전체 경로 회전)
        cos_offset = math.cos(self.heading_offset)
        sin_offset = math.sin(self.heading_offset)
        rotated_x = rel_x * cos_offset - rel_y * sin_offset
        rotated_y = rel_x * sin_offset + rel_y * cos_offset
        
        return {
            "x": 0 + rotated_x,  # GPS 원점으로 다시 이동
            "y": 0 + rotated_y,
            "z": point["z"],
            "timestamp": point["timestamp"]
        }
    
    def publish_markers(self, event):
        """마커 발행"""
        self.visualize_fasterlio_path()
        self.visualize_gps_path()
        self.visualize_global_waypoints()
    
    def visualize_fasterlio_path(self):
        """FasterLIO 경로 시각화 (초록색)"""
        if not self.fasterlio_trajectory:
            return
            
        line_marker = Marker()
        line_marker.header.frame_id = "odom"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "fasterlio_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 2.0
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0  # 초록색
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        points = []
        for pt in self.fasterlio_trajectory:
            points.append(Point(x=pt["x"], y=pt["y"], z=pt["z"]))
        
        line_marker.points = points
        self.path_marker_pub.publish(line_marker)
    
    def visualize_gps_path(self):
        """GPS 경로 시각화 (파란색)"""
        if not self.gps_trajectory:
            return
            
        line_marker = Marker()
        line_marker.header.frame_id = "odom"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "gps_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 2.0
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0  # 파란색
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        points = []
        for gps_pt in self.gps_trajectory:
            x, y = self.gps_to_local_utm(gps_pt["lat"], gps_pt["lon"])
            points.append(Point(x=x, y=y, z=0))
        
        line_marker.points = points
        self.gps_path_pub.publish(line_marker)
    
    def visualize_global_waypoints(self):
        """GPS 웨이포인트 시각화 (빨간색)"""
        marker_array = MarkerArray()
        
        # 기존 마커 삭제
        delete_marker = Marker()
        delete_marker.header.frame_id = "odom"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "global_waypoints"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        if not self.latest_waypoints:
            self.waypoints_marker_pub.publish(marker_array)
            return
        
        # 연결선
        line_marker = Marker()
        line_marker.header.frame_id = "odom"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "global_waypoints"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 3.0
        line_marker.color.r = 1.0  # 빨간색
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        
        points = []
        for wp in self.latest_waypoints:
            x, y = self.gps_to_local_utm(wp["lat"], wp["lon"])
            points.append(Point(x=x, y=y, z=0))
        
        line_marker.points = points
        marker_array.markers.append(line_marker)
        
        # 웨이포인트 큐브들
        for i, wp in enumerate(self.latest_waypoints):
            x, y = self.gps_to_local_utm(wp["lat"], wp["lon"])
            
            cube = Marker()
            cube.header.frame_id = "odom"
            cube.header.stamp = rospy.Time.now()
            cube.ns = "global_waypoints"
            cube.id = i + 1
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = 0
            cube.pose.orientation.w = 1.0
            cube.scale.x = 4.0
            cube.scale.y = 4.0
            cube.scale.z = 1.0
            cube.color.r = 1.0
            cube.color.g = 1.0
            cube.color.b = 0.0  # 노란색
            cube.color.a = 1.0
            
            marker_array.markers.append(cube)
        
        self.waypoints_marker_pub.publish(marker_array)
    
    def broadcast_tf(self, event):
        """TF 브로드캐스트"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)
    
    def publish_gps_data(self, event):
        """GPS 데이터 발행"""
        if self.gps_origin:
            gps_data = {"latitude": self.gps_origin["lat"], "longitude": self.gps_origin["lon"]}
            self.gps_pub.publish(json.dumps(gps_data))

if __name__ == '__main__':
    try:
        visualizer = SimplifiedPathVisualizer()
        rospy.loginfo("🎉 경로 시각화 시스템 실행 중...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 시스템 종료")
    except Exception as e:
        rospy.logerr(f"❌ 시스템 오류: {e}")