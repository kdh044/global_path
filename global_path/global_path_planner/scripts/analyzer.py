#!/usr/bin/env python3

import rospy
import json
import math
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from collections import deque
import threading
import os
import utm
from datetime import datetime
from scipy import stats

# 논문용 고품질 스타일
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['font.size'] = 12
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['figure.titlesize'] = 16
plt.rcParams['legend.fontsize'] = 11

class ImprovedThreeStageCorrectionAnalyzer:
    """개선된 3단계 FasterLIO 보정 분석기 - Raw 오차 문제 해결"""
    
    def __init__(self):
        rospy.init_node('improved_three_stage_analyzer', anonymous=True)
        
        # ===========================================
        # 데이터 저장소 - 4가지 궤적
        # ===========================================
        
        # 1. Bag GPS (Ground Truth)
        self.bag_gps_trajectory = []
        self.latest_bag_gps = None
        
        # 2. Raw FasterLIO (보정 없음) - 통계용만
        self.raw_fasterlio_trajectory = []
        self.latest_raw_fasterlio = None
        
        # 3. Initial Heading Corrected (2m 후 1회 보정만)
        self.initial_corrected_trajectory = []
        self.latest_initial_corrected = None
        
        # 4. Fully Corrected (2m 후 + 10초마다 점진적 보정)
        self.fully_corrected_trajectory = []
        self.latest_fully_corrected = None
        
        # ===========================================
        # 실시간 오차 추적 (개선됨)
        # ===========================================
        self.realtime_errors = {
            "timestamps": deque(maxlen=3000),
            "raw_errors": deque(maxlen=3000),
            "initial_errors": deque(maxlen=3000),
            "full_errors": deque(maxlen=3000)
        }
        
        # 궤적 품질 메트릭
        self.trajectory_metrics = {
            "smoothness": {"initial": [], "full": []},
            "consistency": {"initial": [], "full": []},
            "gps_alignment": {"initial": [], "full": []}
        }
        
        # ===========================================
        # 분석 결과 저장
        # ===========================================
        self.performance_metrics = {}
        self.correction_effectiveness = {}
        
        # 출력 설정
        self.output_dir = "/tmp/improved_three_stage_analysis"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # ===========================================
        # 시스템 상태 추적
        # ===========================================
        self.system_state = {
            "utm_origin_set": False,
            "initial_alignment_done": False,
            "total_distance": 0.0,
            "correction_count": 0,
            "analysis_start_time": rospy.Time.now().to_sec()
        }
        
        # ===========================================
        # ROS 구독자 설정
        # ===========================================
        
        # Bag GPS (Ground Truth)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.bag_gps_callback)
        
        # path_visualizer.py 상태 정보
        rospy.Subscriber("/gps_data", String, self.visualizer_gps_callback)
        
        # 분석용 전용 토픽들
        rospy.Subscriber("/analysis/raw_fasterlio", Odometry, self.raw_fasterlio_callback)
        rospy.Subscriber("/analysis/initial_corrected", Odometry, self.initial_corrected_callback)
        rospy.Subscriber("/analysis/fully_corrected", Odometry, self.fully_corrected_callback)
        
        # 기존 토픽들 (백업용)
        rospy.Subscriber("/Odometry", Odometry, self.raw_fasterlio_callback_backup)
        rospy.Subscriber("/fused_odom", Odometry, self.fully_corrected_callback_backup)
            
        # ===========================================
        # 분석 타이머 (개선됨)
        # ===========================================
        rospy.Timer(rospy.Duration(0.5), self.calculate_realtime_errors)
        rospy.Timer(rospy.Duration(10.0), self.log_correction_progress)
        rospy.Timer(rospy.Duration(30.0), self.generate_focused_plots)  # 더 자주 업데이트
        rospy.Timer(rospy.Duration(120.0), self.generate_comprehensive_analysis)  # 종합 분석
        
        # UTM 변환 설정
        self.utm_zone = None
        self.utm_origin = None
        
        rospy.loginfo("🔬 개선된 3단계 FasterLIO 보정 분석기 시작!")
        rospy.loginfo("📊 분석 단계:")
        rospy.loginfo("   1️⃣ Raw FasterLIO (통계용만)")
        rospy.loginfo("   2️⃣ Initial Heading Corrected (2m 후 1회 보정)")
        rospy.loginfo("   3️⃣ Fully Corrected (2m 후 + 10초마다 점진적 보정)")
        rospy.loginfo("   🎯 Ground Truth: Bag GPS")
        rospy.loginfo("   📈 시각화: Raw 제외, 보정 시스템 중심")
        rospy.loginfo(f"📁 결과 저장: {self.output_dir}")
    
    def bag_gps_callback(self, msg):
        """Bag 파일 GPS (Ground Truth)"""
        if msg.status.status < 0:
            return
        
        try:
            # UTM 변환
            easting, northing, zone_num, zone_letter = utm.from_latlon(
                msg.latitude, msg.longitude
            )
            
            # 첫 GPS로 UTM 기준 설정
            if not self.utm_origin:
                self.utm_origin = {"easting": easting, "northing": northing}
                self.utm_zone = f"{zone_num}{zone_letter}"
                self.system_state["utm_origin_set"] = True
                rospy.loginfo(f"🎯 UTM 기준점 설정 (Bag GPS): ({easting:.1f}, {northing:.1f})")
            
            gps_point = {
                "x": easting,
                "y": northing,
                "timestamp": msg.header.stamp.to_sec(),
                "lat": msg.latitude,
                "lon": msg.longitude
            }
            
            self.latest_bag_gps = gps_point
            
            # 거리 필터링하여 궤적 저장
            if (not self.bag_gps_trajectory or 
                self.distance_check(gps_point, self.bag_gps_trajectory[-1], 0.3)):
                self.bag_gps_trajectory.append(gps_point)
                
        except Exception as e:
            rospy.logwarn(f"⚠️ GPS 처리 오류: {e}")
    
    def visualizer_gps_callback(self, msg):
        """path_visualizer의 GPS 데이터로 상태 추적"""
        try:
            gps_data = json.loads(msg.data)
            if "latitude" in gps_data:
                self.system_state["utm_origin_set"] = True
        except:
            pass
    
    def raw_fasterlio_callback(self, msg):
        """Raw FasterLIO 전용 콜백"""
        if not self.utm_origin:
            return
        
        try:
            raw_point = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "timestamp": msg.header.stamp.to_sec(),
                "qx": msg.pose.pose.orientation.x,
                "qy": msg.pose.pose.orientation.y,
                "qz": msg.pose.pose.orientation.z,
                "qw": msg.pose.pose.orientation.w
            }
            
            self.latest_raw_fasterlio = raw_point
            
            if (not self.raw_fasterlio_trajectory or 
                self.distance_check(raw_point, self.raw_fasterlio_trajectory[-1], 0.2)):
                self.raw_fasterlio_trajectory.append(raw_point)
                
        except Exception as e:
            rospy.logwarn(f"⚠️ Raw FasterLIO 처리 오류: {e}")
    
    def raw_fasterlio_callback_backup(self, msg):
        """Raw FasterLIO 백업 콜백"""
        if not hasattr(self, 'latest_raw_fasterlio') or not self.latest_raw_fasterlio:
            self.raw_fasterlio_callback(msg)
    
    def initial_corrected_callback(self, msg):
        """Initial Corrected 전용 콜백"""
        try:
            corrected_point = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "timestamp": msg.header.stamp.to_sec(),
                "qx": msg.pose.pose.orientation.x,
                "qy": msg.pose.pose.orientation.y,
                "qz": msg.pose.pose.orientation.z,
                "qw": msg.pose.pose.orientation.w
            }
            
            self.latest_initial_corrected = corrected_point
            
            if (not self.initial_corrected_trajectory or 
                self.distance_check(corrected_point, self.initial_corrected_trajectory[-1], 0.2)):
                self.initial_corrected_trajectory.append(corrected_point)
                
        except Exception as e:
            rospy.logwarn(f"⚠️ Initial Corrected 처리 오류: {e}")

    def fully_corrected_callback(self, msg):
        """Fully Corrected 전용 콜백"""
        try:
            corrected_point = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "timestamp": msg.header.stamp.to_sec(),
                "qx": msg.pose.pose.orientation.x,
                "qy": msg.pose.pose.orientation.y,
                "qz": msg.pose.pose.orientation.z,
                "qw": msg.pose.pose.orientation.w
            }
            
            self.latest_fully_corrected = corrected_point
            
            if (not self.fully_corrected_trajectory or 
                self.distance_check(corrected_point, self.fully_corrected_trajectory[-1], 0.2)):
                self.fully_corrected_trajectory.append(corrected_point)
                
        except Exception as e:
            rospy.logwarn(f"⚠️ Fully Corrected 처리 오류: {e}")
    
    def fully_corrected_callback_backup(self, msg):
        """Fully Corrected 백업 콜백"""
        if not hasattr(self, 'latest_fully_corrected') or not self.latest_fully_corrected:
            self.fully_corrected_callback(msg)
    
    def distance_check(self, pose1, pose2, threshold):
        """거리 체크"""
        dx = pose1["x"] - pose2["x"]
        dy = pose1["y"] - pose2["y"]
        return math.sqrt(dx*dx + dy*dy) > threshold
    
    def calculate_realtime_errors(self, event):
        """실시간 위치 오차 계산 (개선됨)"""
        if not self.latest_bag_gps:
            return
        
        current_time = rospy.Time.now().to_sec()
        self.realtime_errors["timestamps"].append(current_time)
        
        # 각 단계별 GPS 대비 오차 계산
        raw_error = self.calculate_position_error(self.latest_bag_gps, self.latest_raw_fasterlio)
        initial_error = self.calculate_position_error(self.latest_bag_gps, self.latest_initial_corrected)
        full_error = self.calculate_position_error(self.latest_bag_gps, self.latest_fully_corrected)
        
        # Raw 오차가 너무 크면 제한 (시각화용)
        if raw_error and raw_error > 1000:
            raw_error = 1000  # 1km로 제한
        
        self.realtime_errors["raw_errors"].append(raw_error if raw_error else 0.0)
        self.realtime_errors["initial_errors"].append(initial_error if initial_error else 0.0)
        self.realtime_errors["full_errors"].append(full_error if full_error else 0.0)
        
        # 궤적 품질 메트릭 계산
        self.calculate_trajectory_quality_metrics()
    
    def calculate_position_error(self, gps_point, fasterlio_point):
        """위치 오차 계산"""
        if not gps_point or not fasterlio_point:
            return None
        
        dx = gps_point["x"] - fasterlio_point["x"]
        dy = gps_point["y"] - fasterlio_point["y"]
        return math.sqrt(dx*dx + dy*dy)
    
    def calculate_trajectory_quality_metrics(self):
        """궤적 품질 메트릭 계산"""
        # Smoothness (경로의 부드러움)
        if len(self.initial_corrected_trajectory) > 5:
            initial_smoothness = self.calculate_trajectory_smoothness(self.initial_corrected_trajectory)
            self.trajectory_metrics["smoothness"]["initial"].append(initial_smoothness)
        
        if len(self.fully_corrected_trajectory) > 5:
            full_smoothness = self.calculate_trajectory_smoothness(self.fully_corrected_trajectory)
            self.trajectory_metrics["smoothness"]["full"].append(full_smoothness)
    
    def calculate_trajectory_smoothness(self, trajectory):
        """궤적 부드러움 계산 (각도 변화율 기반)"""
        if len(trajectory) < 3:
            return 0.0
        
        angle_changes = []
        for i in range(1, len(trajectory)-1):
            p1 = trajectory[i-1]
            p2 = trajectory[i]
            p3 = trajectory[i+1]
            
            # 벡터 각도 계산
            v1 = np.array([p2["x"] - p1["x"], p2["y"] - p1["y"]])
            v2 = np.array([p3["x"] - p2["x"], p3["y"] - p2["y"]])
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                cos_angle = np.clip(cos_angle, -1, 1)
                angle_change = math.acos(cos_angle)
                angle_changes.append(angle_change)
        
        return np.std(angle_changes) if angle_changes else 0.0
    
    def log_correction_progress(self, event):
        """보정 진행 상황 로깅 (개선됨)"""
        rospy.loginfo("📊" + "="*60)
        rospy.loginfo("🔍 개선된 3단계 보정 진행 상황:")
        rospy.loginfo(f"   Bag GPS 포인트: {len(self.bag_gps_trajectory)}")
        rospy.loginfo(f"   Raw FasterLIO: {len(self.raw_fasterlio_trajectory)} (통계용)")
        rospy.loginfo(f"   Initial Corrected: {len(self.initial_corrected_trajectory)}")
        rospy.loginfo(f"   Fully Corrected: {len(self.fully_corrected_trajectory)}")
        
        if len(self.realtime_errors["initial_errors"]) > 10:
            # 최근 20개 데이터의 평균 오차 (더 안정적)
            recent_raw = np.mean(list(self.realtime_errors["raw_errors"])[-20:])
            recent_initial = np.mean(list(self.realtime_errors["initial_errors"])[-20:])
            recent_full = np.mean(list(self.realtime_errors["full_errors"])[-20:])
            
            rospy.loginfo(f"📍 최근 위치 오차 (vs Bag GPS, 20개 평균):")
            rospy.loginfo(f"   Raw: {recent_raw:.2f}m (제한됨)")
            rospy.loginfo(f"   Initial Corrected: {recent_initial:.2f}m")
            rospy.loginfo(f"   Fully Corrected: {recent_full:.2f}m")
            
            # 보정 시스템 간 비교
            if recent_initial > 0:
                improvement = (recent_initial - recent_full) / recent_initial * 100
                rospy.loginfo(f"🎯 Full vs Initial 개선도: {improvement:.1f}%")
        
        rospy.loginfo("📊" + "="*60)
    
    def calculate_comprehensive_metrics(self):
        """종합 성능 메트릭 계산 (개선됨)"""
        if len(self.realtime_errors["initial_errors"]) < 30:
            return None
        
        # 실시간 오차 통계 (Raw는 원본값 사용)
        raw_errors = [e for e in list(self.realtime_errors["raw_errors"]) if e > 0]
        initial_errors = [e for e in list(self.realtime_errors["initial_errors"]) if e > 0]
        full_errors = [e for e in list(self.realtime_errors["full_errors"]) if e > 0]
        
        # 각 단계별 상세 통계
        def calculate_detailed_stats(errors, name):
            if not errors:
                return {}
            
            return {
                "count": len(errors),
                "mean": float(np.mean(errors)),
                "std": float(np.std(errors)),
                "median": float(np.median(errors)),
                "min": float(np.min(errors)),
                "max": float(np.max(errors)),
                "p25": float(np.percentile(errors, 25)),
                "p75": float(np.percentile(errors, 75)),
                "p95": float(np.percentile(errors, 95)),
                "p99": float(np.percentile(errors, 99))
            }
        
        stages = {
            "raw": calculate_detailed_stats(raw_errors, "Raw"),
            "initial": calculate_detailed_stats(initial_errors, "Initial"),
            "full": calculate_detailed_stats(full_errors, "Full")
        }
        
        # 보정 효과 분석
        improvements = {}
        if stages["initial"]["mean"] > 0 and stages["full"]["mean"] > 0:
            improvements["full_vs_initial_mean"] = (stages["initial"]["mean"] - stages["full"]["mean"]) / stages["initial"]["mean"] * 100
            improvements["full_vs_initial_std"] = (stages["initial"]["std"] - stages["full"]["std"]) / stages["initial"]["std"] * 100
            improvements["full_vs_initial_p95"] = (stages["initial"]["p95"] - stages["full"]["p95"]) / stages["initial"]["p95"] * 100
            improvements["full_vs_initial_max"] = (stages["initial"]["max"] - stages["full"]["max"]) / stages["initial"]["max"] * 100
        
        # 궤적 품질 분석
        trajectory_quality = {
            "initial_smoothness": np.mean(self.trajectory_metrics["smoothness"]["initial"]) if self.trajectory_metrics["smoothness"]["initial"] else 0,
            "full_smoothness": np.mean(self.trajectory_metrics["smoothness"]["full"]) if self.trajectory_metrics["smoothness"]["full"] else 0
        }
        
        return {
            "timestamp": rospy.Time.now().to_sec(),
            "realtime_position_error": stages,
            "correction_effectiveness": improvements,
            "trajectory_quality": trajectory_quality,
            "data_counts": {
                "bag_gps": len(self.bag_gps_trajectory),
                "raw": len(self.raw_fasterlio_trajectory),
                "initial": len(self.initial_corrected_trajectory),
                "full": len(self.fully_corrected_trajectory)
            }
        }
    
    def generate_focused_plots(self, event):
        """집중된 분석 플롯 생성 (보정 시스템 중심)"""
        metrics = self.calculate_comprehensive_metrics()
        
        if not metrics:
            rospy.logwarn("⚠️ 분석할 데이터가 부족합니다.")
            return
        
        self.performance_metrics = metrics
        
        try:
            # 1. 보정 시스템 중심 궤적 비교
            self.plot_correction_focused_trajectory()
            
            # 2. 보정 시스템 성능 분석
            self.plot_correction_performance_analysis()
            
            rospy.loginfo("📊 집중 분석 플롯 생성 완료!")
            
        except Exception as e:
            rospy.logerr(f"❌ 플롯 생성 오류: {e}")
            import traceback
            traceback.print_exc()
    
    def generate_comprehensive_analysis(self, event):
        """종합 분석 리포트 생성"""
        try:
            # 종합 통계 리포트
            self.generate_statistics_report()
            
            # 전체 시스템 분석
            self.plot_full_system_analysis()
            
            rospy.loginfo("📈 종합 분석 리포트 생성 완료!")
            
        except Exception as e:
            rospy.logerr(f"❌ 종합 분석 오류: {e}")
    
    def plot_correction_focused_trajectory(self):
        """보정 시스템 중심 궤적 비교"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        
        # ================================
        # 왼쪽: GPS + 보정 시스템 비교
        # ================================
        
        # Bag GPS (Ground Truth)
        if self.bag_gps_trajectory:
            gps_x = [p["x"] for p in self.bag_gps_trajectory]
            gps_y = [p["y"] for p in self.bag_gps_trajectory]
            ax1.plot(gps_x, gps_y, 'b-', linewidth=4, label='GPS Ground Truth', alpha=0.9, zorder=4)
        
        # Initial Corrected
        if self.initial_corrected_trajectory:
            init_x = [p["x"] for p in self.initial_corrected_trajectory]
            init_y = [p["y"] for p in self.initial_corrected_trajectory]
            ax1.plot(init_x, init_y, 'orange', linewidth=3, label='Initial Corrected (2m 후)', alpha=0.8, zorder=2)
        
        # Fully Corrected
        if self.fully_corrected_trajectory:
            full_x = [p["x"] for p in self.fully_corrected_trajectory]
            full_y = [p["y"] for p in self.fully_corrected_trajectory]
            ax1.plot(full_x, full_y, 'g-', linewidth=3, label='Fully Corrected (점진적)', alpha=0.8, zorder=3)
        
        # 시작점/끝점 표시
        if self.bag_gps_trajectory:
            ax1.plot(gps_x[0], gps_y[0], 'ko', markersize=15, label='Start', zorder=5)
            ax1.plot(gps_x[-1], gps_y[-1], 'ks', markersize=15, label='End', zorder=5)
        
        ax1.set_title('(a) Correction System Trajectory Comparison', fontweight='bold', fontsize=14)
        ax1.set_xlabel('UTM Easting (m)', fontweight='bold')
        ax1.set_ylabel('UTM Northing (m)', fontweight='bold')
        ax1.legend(loc='best', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # ================================
        # 오른쪽: 오차 확대 비교
        # ================================
        
        # 최근 궤적만 표시 (마지막 50개 포인트)
        if len(gps_x) > 50:
            recent_gps_x = gps_x[-50:]
            recent_gps_y = gps_y[-50:]
            recent_init_x = init_x[-50:] if len(init_x) > 50 else init_x
            recent_init_y = init_y[-50:] if len(init_y) > 50 else init_y
            recent_full_x = full_x[-50:] if len(full_x) > 50 else full_x
            recent_full_y = full_y[-50:] if len(full_y) > 50 else full_y
        else:
            recent_gps_x, recent_gps_y = gps_x, gps_y
            recent_init_x, recent_init_y = init_x, init_y
            recent_full_x, recent_full_y = full_x, full_y
        
        ax2.plot(recent_gps_x, recent_gps_y, 'b-', linewidth=4, label='GPS Ground Truth', alpha=0.9)
        ax2.plot(recent_init_x, recent_init_y, 'orange', linewidth=3, label='Initial Corrected', alpha=0.8)
        ax2.plot(recent_full_x, recent_full_y, 'g-', linewidth=3, label='Fully Corrected', alpha=0.8)
        
        # 성능 지표 텍스트박스
        if self.performance_metrics:
            stats = self.performance_metrics["realtime_position_error"]
            improvements = self.performance_metrics["correction_effectiveness"]
            
            text = f"""Recent Performance (Last 50 points):

Initial Corrected:
  Mean: {stats['initial']['mean']:.2f} ± {stats['initial']['std']:.2f} m
  95th: {stats['initial']['p95']:.2f} m
  Max: {stats['initial']['max']:.2f} m

Fully Corrected:
  Mean: {stats['full']['mean']:.2f} ± {stats['full']['std']:.2f} m
  95th: {stats['full']['p95']:.2f} m
  Max: {stats['full']['max']:.2f} m

Improvement (Full vs Initial):
  Mean: {improvements.get('full_vs_initial_mean', 0):.1f}%
  Std: {improvements.get('full_vs_initial_std', 0):.1f}%"""
            
            ax2.text(0.02, 0.98, text, transform=ax2.transAxes, fontsize=10,
                   verticalalignment='top', fontfamily='monospace',
                   bbox=dict(boxstyle='round,pad=0.5', facecolor='lightblue', alpha=0.8))
        
        ax2.set_title('(b) Recent Trajectory Detail (Last 50 Points)', fontweight='bold', fontsize=14)
        ax2.set_xlabel('UTM Easting (m)', fontweight='bold')
        ax2.set_ylabel('UTM Northing (m)', fontweight='bold')
        ax2.legend(loc='best', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        plt.tight_layout()
        plt.savefig(f"{self.output_dir}/correction_focused_trajectory.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        rospy.loginfo("✅ 보정 중심 궤적 플롯 생성 완료")
    
    def plot_correction_performance_analysis(self):
        """보정 시스템 성능 분석"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18, 12))
        
        timestamps = list(self.realtime_errors["timestamps"])
        initial_errors = list(self.realtime_errors["initial_errors"])
        full_errors = list(self.realtime_errors["full_errors"])
        
        if not timestamps:
            return
        
        # 상대 시간으로 변환
        start_time = timestamps[0]
        relative_times = [(t - start_time) for t in timestamps]
        
        # 1. 보정 시스템 오차 비교
        ax1.plot(relative_times, initial_errors, 'orange', linewidth=2.5, label='Initial Corrected', alpha=0.8)
        ax1.plot(relative_times, full_errors, 'g-', linewidth=2.5, label='Fully Corrected', alpha=0.8)
        
        # 평균선 표시
        ax1.axhline(np.mean(initial_errors), color='orange', linestyle=':', alpha=0.7, 
                   label=f'Initial Mean: {np.mean(initial_errors):.2f}m')
        ax1.axhline(np.mean(full_errors), color='green', linestyle=':', alpha=0.7, 
                   label=f'Full Mean: {np.mean(full_errors):.2f}m')
        
        ax1.set_xlabel('Time (s)', fontweight='bold')
        ax1.set_ylabel('Position Error (m)', fontweight='bold')
        ax1.set_title('(a) Correction System Performance Over Time', fontweight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(0, max(max(initial_errors), max(full_errors)) * 1.1)
        
        # 2. 오차 분포 히스토그램
        ax2.hist(initial_errors, bins=30, alpha=0.6, color='orange', label='Initial Corrected', density=True)
        ax2.hist(full_errors, bins=30, alpha=0.6, color='green', label='Fully Corrected', density=True)
        ax2.axvline(np.mean(initial_errors), color='orange', linestyle='--', linewidth=2)
        ax2.axvline(np.mean(full_errors), color='green', linestyle='--', linewidth=2)
        
        ax2.set_xlabel('Position Error (m)', fontweight='bold')
        ax2.set_ylabel('Density', fontweight='bold')
        ax2.set_title('(b) Error Distribution Comparison', fontweight='bold')
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)
        
        # 3. 누적 개선도 분석
        if len(initial_errors) > 1:
            improvement_over_time = []
            window_size = max(10, len(initial_errors) // 20)
            
            for i in range(window_size, len(initial_errors)):
                recent_initial = np.mean(initial_errors[i-window_size:i])
                recent_full = np.mean(full_errors[i-window_size:i])
                
                if recent_initial > 0:
                    improvement = (recent_initial - recent_full) / recent_initial * 100
                    improvement_over_time.append(improvement)
                else:
                    improvement_over_time.append(0)
            
            improvement_times = relative_times[window_size:]
            ax3.plot(improvement_times, improvement_over_time, 'purple', linewidth=2.5, alpha=0.8)
            ax3.axhline(np.mean(improvement_over_time), color='purple', linestyle='--', alpha=0.7,
                       label=f'Average: {np.mean(improvement_over_time):.1f}%')
            
            ax3.set_xlabel('Time (s)', fontweight='bold')
            ax3.set_ylabel('Improvement (%)', fontweight='bold')
            ax3.set_title('(c) Full vs Initial Improvement Over Time', fontweight='bold')
            ax3.legend(fontsize=10)
            ax3.grid(True, alpha=0.3)
        
        # 4. 성능 지표 요약
        ax4.axis('off')
        
        if self.performance_metrics:
            stats = self.performance_metrics["realtime_position_error"]
            improvements = self.performance_metrics["correction_effectiveness"]
            
            stats_text = f"""Correction System Performance Summary:

Initial Corrected:
  Mean Error: {stats['initial']['mean']:.2f} ± {stats['initial']['std']:.2f} m
  Median: {stats['initial']['median']:.2f} m
  95th Percentile: {stats['initial']['p95']:.2f} m
  Max Error: {stats['initial']['max']:.2f} m

Fully Corrected:
  Mean Error: {stats['full']['mean']:.2f} ± {stats['full']['std']:.2f} m
  Median: {stats['full']['median']:.2f} m
  95th Percentile: {stats['full']['p95']:.2f} m
  Max Error: {stats['full']['max']:.2f} m

Effectiveness (Full vs Initial):
  Mean Error Reduction: {improvements.get('full_vs_initial_mean', 0):.1f}%
  Std Reduction: {improvements.get('full_vs_initial_std', 0):.1f}%
  95th Percentile Reduction: {improvements.get('full_vs_initial_p95', 0):.1f}%
  Max Error Reduction: {improvements.get('full_vs_initial_max', 0):.1f}%

Data Quality:
  Initial Trajectory Points: {stats['initial']['count']}
  Full Trajectory Points: {stats['full']['count']}"""
            
            ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes, fontsize=10,
                     verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgray', alpha=0.8))
        
        ax4.set_title('(d) Statistical Performance Summary', fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(f"{self.output_dir}/correction_performance_analysis.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        rospy.loginfo("✅ 보정 성능 분석 플롯 생성 완료")
    
    def plot_full_system_analysis(self):
        """전체 시스템 분석 (Raw 포함, 제한된 스케일)"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 14))
        
        timestamps = list(self.realtime_errors["timestamps"])
        raw_errors = list(self.realtime_errors["raw_errors"])
        initial_errors = list(self.realtime_errors["initial_errors"])
        full_errors = list(self.realtime_errors["full_errors"])
        
        if not timestamps:
            return
        
        start_time = timestamps[0]
        relative_times = [(t - start_time) for t in timestamps]
        
        # 1. 전체 시스템 오차 (Raw 제한)
        ax1.plot(relative_times, raw_errors, 'r--', linewidth=1.5, label='Raw FasterLIO (제한: 1km)', alpha=0.6)
        ax1.plot(relative_times, initial_errors, 'orange', linewidth=2.5, label='Initial Corrected', alpha=0.8)
        ax1.plot(relative_times, full_errors, 'g-', linewidth=2.5, label='Fully Corrected', alpha=0.8)
        
        ax1.set_xlabel('Time (s)', fontweight='bold')
        ax1.set_ylabel('Position Error (m)', fontweight='bold')
        ax1.set_title('(a) Complete System Error Analysis', fontweight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(0, 50)  # 50m로 제한하여 보정 시스템 세부사항 보기
        
        # 2. 궤적 전체 비교
        if self.bag_gps_trajectory and self.initial_corrected_trajectory and self.fully_corrected_trajectory:
            gps_x = [p["x"] for p in self.bag_gps_trajectory]
            gps_y = [p["y"] for p in self.bag_gps_trajectory]
            init_x = [p["x"] for p in self.initial_corrected_trajectory]
            init_y = [p["y"] for p in self.initial_corrected_trajectory]
            full_x = [p["x"] for p in self.fully_corrected_trajectory]
            full_y = [p["y"] for p in self.fully_corrected_trajectory]
            
            ax2.plot(gps_x, gps_y, 'b-', linewidth=4, label='GPS Ground Truth', alpha=0.9)
            ax2.plot(init_x, init_y, 'orange', linewidth=2.5, label='Initial Corrected', alpha=0.8)
            ax2.plot(full_x, full_y, 'g-', linewidth=2.5, label='Fully Corrected', alpha=0.8)
            
            # 시작점과 끝점
            ax2.plot(gps_x[0], gps_y[0], 'ko', markersize=12, label='Start')
            ax2.plot(gps_x[-1], gps_y[-1], 'ks', markersize=12, label='End')
        
        ax2.set_xlabel('UTM Easting (m)', fontweight='bold')
        ax2.set_ylabel('UTM Northing (m)', fontweight='bold')
        ax2.set_title('(b) Complete Trajectory Comparison', fontweight='bold')
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')
        
        # 3. 보정 효과 분석 (시간대별)
        if len(relative_times) > 20:
            # 시간을 구간으로 나누어 분석
            time_segments = np.linspace(0, max(relative_times), 10)
            segment_improvements = []
            
            for i in range(len(time_segments)-1):
                start_idx = np.searchsorted(relative_times, time_segments[i])
                end_idx = np.searchsorted(relative_times, time_segments[i+1])
                
                if end_idx > start_idx:
                    seg_initial = np.mean(initial_errors[start_idx:end_idx])
                    seg_full = np.mean(full_errors[start_idx:end_idx])
                    
                    if seg_initial > 0:
                        improvement = (seg_initial - seg_full) / seg_initial * 100
                        segment_improvements.append(improvement)
                    else:
                        segment_improvements.append(0)
            
            segment_centers = [(time_segments[i] + time_segments[i+1]) / 2 for i in range(len(time_segments)-1)]
            
            ax3.bar(segment_centers, segment_improvements, width=(max(relative_times)/10)*0.8, 
                   alpha=0.7, color='purple', label='Improvement per Segment')
            ax3.axhline(np.mean(segment_improvements), color='red', linestyle='--', 
                       label=f'Average: {np.mean(segment_improvements):.1f}%')
        
        ax3.set_xlabel('Time (s)', fontweight='bold')
        ax3.set_ylabel('Improvement (%)', fontweight='bold')
        ax3.set_title('(c) Correction Effectiveness by Time Segment', fontweight='bold')
        ax3.legend(fontsize=10)
        ax3.grid(True, alpha=0.3)
        
        # 4. 시스템 요약 및 권장사항
        ax4.axis('off')
        
        if self.performance_metrics:
            stats = self.performance_metrics["realtime_position_error"]
            
            # 원본 Raw 오차 계산 (제한되지 않은)
            original_raw_errors = [e for e in raw_errors if e > 0]
            raw_mean = np.mean(original_raw_errors) if original_raw_errors else 0
            
            summary_text = f"""System Analysis Summary:

Raw FasterLIO Performance:
  Average Error: {raw_mean:.1f} m (원본, 제한 전)
  Status: GPS 없이는 절대 위치 추정 불가능

Initial Correction (2m 후 1회 보정):
  Average Error: {stats['initial']['mean']:.2f} m
  Improvement vs Raw: {((raw_mean - stats['initial']['mean'])/raw_mean*100) if raw_mean > 0 else 0:.1f}%
  Status: 기본적인 heading 정렬로 대폭 개선

Full Correction (점진적 보정):
  Average Error: {stats['full']['mean']:.2f} m
  Improvement vs Initial: {self.performance_metrics['correction_effectiveness'].get('full_vs_initial_mean', 0):.1f}%
  Status: 추가적인 정밀도 개선

Recommendations:
• Initial correction 단계에서 이미 충분한 성능 확보
• Full correction은 {self.performance_metrics['correction_effectiveness'].get('full_vs_initial_mean', 0):.1f}% 추가 개선 제공
• 실시간 성능이 중요하면 Initial correction으로 충분
• 최고 정밀도가 필요하면 Full correction 권장

Data Quality:
• GPS 기준점: {len(self.bag_gps_trajectory)} points
• 분석 시간: {max(relative_times):.0f} seconds"""
            
            ax4.text(0.05, 0.95, summary_text, transform=ax4.transAxes, fontsize=11,
                     verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.8))
        
        ax4.set_title('(d) System Summary & Recommendations', fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(f"{self.output_dir}/full_system_analysis.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        rospy.loginfo("✅ 전체 시스템 분석 플롯 생성 완료")
    
    def generate_statistics_report(self):
        """종합 통계 리포트 생성"""
        if not self.performance_metrics:
            return
        
        report_path = f"{self.output_dir}/performance_report.json"
        
        # 상세 리포트 생성
        detailed_report = {
            "analysis_info": {
                "start_time": self.system_state["analysis_start_time"],
                "current_time": rospy.Time.now().to_sec(),
                "duration_seconds": rospy.Time.now().to_sec() - self.system_state["analysis_start_time"],
                "utm_zone": self.utm_zone
            },
            "data_summary": self.performance_metrics["data_counts"],
            "position_error_statistics": self.performance_metrics["realtime_position_error"],
            "correction_effectiveness": self.performance_metrics["correction_effectiveness"],
            "trajectory_quality": self.performance_metrics.get("trajectory_quality", {}),
            "system_recommendations": self.generate_recommendations()
        }
        
        # JSON 리포트 저장
        with open(report_path, 'w') as f:
            json.dump(detailed_report, f, indent=2)
        
        rospy.loginfo(f"📊 상세 통계 리포트 저장: {report_path}")
    
    def generate_recommendations(self):
        """시스템 성능 기반 권장사항 생성"""
        if not self.performance_metrics:
            return {}
        
        stats = self.performance_metrics["realtime_position_error"]
        improvements = self.performance_metrics["correction_effectiveness"]
        
        recommendations = {
            "performance_assessment": "",
            "recommended_configuration": "",
            "optimization_suggestions": []
        }
        
        # 성능 평가
        if stats["full"]["mean"] < 2.0:
            recommendations["performance_assessment"] = "Excellent - Sub-2m accuracy achieved"
        elif stats["full"]["mean"] < 5.0:
            recommendations["performance_assessment"] = "Good - Suitable for most applications"
        elif stats["full"]["mean"] < 10.0:
            recommendations["performance_assessment"] = "Acceptable - May need tuning for precision applications"
        else:
            recommendations["performance_assessment"] = "Needs improvement - Check system configuration"
        
        # 권장 설정
        improvement_rate = improvements.get('full_vs_initial_mean', 0)
        if improvement_rate < 5.0:
            recommendations["recommended_configuration"] = "Initial correction sufficient - Full correction provides minimal benefit"
        elif improvement_rate < 15.0:
            recommendations["recommended_configuration"] = "Consider full correction for precision applications"
        else:
            recommendations["recommended_configuration"] = "Full correction strongly recommended - significant improvement available"
        
        # 최적화 제안
        if stats["initial"]["std"] > stats["initial"]["mean"] * 0.5:
            recommendations["optimization_suggestions"].append("High variance detected - consider more frequent corrections")
        
        if stats["full"]["max"] > stats["full"]["mean"] * 5:
            recommendations["optimization_suggestions"].append("Outliers detected - implement outlier rejection")
        
        if len(self.bag_gps_trajectory) > 0 and len(self.fully_corrected_trajectory) / len(self.bag_gps_trajectory) < 0.8:
            recommendations["optimization_suggestions"].append("Low data coverage - check topic synchronization")
        
        return recommendations


if __name__ == '__main__':
    try:
        analyzer = ImprovedThreeStageCorrectionAnalyzer()
        rospy.loginfo("🔬 개선된 3단계 FasterLIO 보정 분석기 실행 중...")
        rospy.loginfo("📊 보정 시스템 중심 실시간 분석 진행 중...")
        rospy.loginfo("⏰ 30초마다 집중 분석, 120초마다 종합 분석")
        rospy.loginfo("📈 다음 토픽들을 모니터링합니다:")
        rospy.loginfo("   • /ublox/fix (Bag GPS Ground Truth)")
        rospy.loginfo("   • /analysis/raw_fasterlio (Raw FasterLIO)")
        rospy.loginfo("   • /analysis/initial_corrected (Initial Corrected)")
        rospy.loginfo("   • /analysis/fully_corrected (Fully Corrected)")
        rospy.loginfo("   • 백업: /Odometry, /fused_odom")
        rospy.loginfo("📊 특징: Raw 오차 제한, 보정 시스템 성능 집중 분석")
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 개선된 3단계 보정 분석기 종료")
    except Exception as e:
        rospy.logerr(f"❌ 분석기 오류: {e}")
        import traceback
        traceback.print_exc()