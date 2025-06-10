# Global Path Planner
Localization 및 Global Path 생성

## 주요 기능

- GPS + Waypoint 기능: 카카오 API를 활용한 웨이포인트 생성 및 자동 삭제 (5m 이내)
- 3단계 FasterLIO 보정 시스템: GPS 기준 점진적 heading drift 보정
- Waypoint 도달(5m 이내) 시 자동 삭제
- 실시간 성능 분석: 궤적 정확도 실시간 모니터링 및 통계 분석
- UTM 좌표계 통합: GPS와 SLAM 데이터의 일관된 좌표계 시각화
- 오프라인 분석 지원: Bag 파일 기반 데이터 재현 및 분석
- 실시간 성능 분석

--- 

## 시스템 구성

### 3단계 FasterLIO 보정 시스템

**실험 조건**: 총 주행거리 969.2m, 약 50분간 실시간 분석

| 단계 | 설명 | 평균 오차 |
|------|------|-----------|
| **Raw FasterLIO** | Odometry | ~344m |
| **Initial Corrected** | 2m 이동 후 1회 GPS 이용하여 heading 정렬 | ~165m |
| **Fully Corrected** |heading 정렬 후 + GPS 센서 이용하여 10초마다 점진적 보정 | ~1.3m |

**요약**:
- **시간에 따른 성능 악화**: Initial Corrected는 110m → 400m+로 오차가 증가
- **일관된 성능 유지**: Fully Corrected는 지속적으로 2-5m 정확도 유지
- **최종 개선도**: Full vs Initial **99.2% 개선**



### 핵심 요소 

1. **FasterLIO SLAM**: LiDAR + IMU 기반 실시간 위치 추정
2. **GPS 보정 시스템**: Heading drift 자동 감지 및 보정
3. **UTM 좌표계 통합**: GPS와 SLAM 좌표의 일관된 처리
4. **실시간 성능 분석**: 보정 효과 실시간 모니터링
5. **카카오 API 연동**: 웹 인터페이스를 통한 경로 계획

### 데이터 흐름
```
Bag 파일 (IMU + LiDAR) → FasterLIO → 3단계 보정 시스템 → /fused_odom → RViz
     ↓                                    ↓
GPS 원점 추출 → path_visualizer.py → UTM 변환 → 성능 분석 → analyzer.py
     ↓
/gps_data → 웹서버 → 카카오 API → /waypoints → RViz 시각화
```

## 사용 방법

### 기본 실행 (2025.6.11 추가)
```bash
# 1. FasterLIO SLAM 실행
roslaunch faster_lio mapping_ouster32.launch

# 2. Bag 파일 재생
rosrun global_path_planner bag_player.py

# 3. 3단계 보정 및 경로 시각화
rosrun global_path_planner path_visualizer.py

# 4. 실시간 성능 분석
rosrun global_path_planner analyzer.py

# 5. 웹 인터페이스
rosrun global_path_planner gps_server.py
```


## 주요 토픽

| 토픽명 | 메시지 타입 | 설명 |
|--------|-------------|------|
| `/Odometry` | nav_msgs/Odometry | FasterLIO SLAM 원본 결과 |
| `/fused_odom` | nav_msgs/Odometry | 3단계 보정 적용 결과 |
| `/ublox/fix` | sensor_msgs/NavSatFix | Bag GPS Ground Truth |
| `/gps_data` | std_msgs/String | GPS 원점 좌표 (JSON) |
| `/waypoints` | std_msgs/String | 카카오 API 경로 (JSON) |
| `/analysis/raw_fasterlio` | nav_msgs/Odometry | 분석용 Raw 데이터 |
| `/analysis/initial_corrected` | nav_msgs/Odometry | 분석용 Initial 보정 데이터 |
| `/analysis/fully_corrected` | nav_msgs/Odometry | 분석용 Full 보정 데이터 |


## 시각화 결과

### 1. ROS 노드 구성 (rqt_graph)
![ROS 노드 구성](rqt.png)


### 2. RViz 경로 시각화
![RViz 경로 시각화](rviz.png)

**색상 구분**:
- **파란색 선**: GPS Ground Truth (Bag 파일)
- **회색 선**: Raw FasterLIO 궤적
- **주황색 선**: Initial Corrected 궤적
- **녹색 선**: Fully Corrected 궤적
- **빨간색 선 + 노란색 큐브**: 카카오 API 웨이포인트


### 성능 분석 결과
- **실시간 오차 모니터링**: GPS 대비 위치 오차 실시간 추적
- **자동 리포트 생성**: JSON/CSV 형태의 상세 분석 데이터
- **시각화 플롯**: 궤적 비교, 오차 분포, 개선도 분석

## 출력 파일

### 분석 결과 (`/tmp/improved_three_stage_analysis/`)
- `correction_focused_trajectory.png`: 보정 시스템 중심 궤적 비교
- `correction_performance_analysis.png`: 보정 성능 상세 분석
- `full_system_analysis.png`: 전체 시스템 종합 분석
- `performance_report.json`: 성능 통계 요약
- `complete_analysis_data.json`: 완전한 궤적 및 시계열 데이터
- `timeseries_analysis.json`: 시계열 분석용 데이터
- 'rqt.png


### CSV 파일 (연구용)
- `gps_ground_truth.csv`: GPS 궤적 데이터
- `raw_fasterlio.csv`: Raw FasterLIO 궤적
- `fully_corrected.csv`: 보정된 궤적
- `realtime_errors.csv`: 시계열 오차 데이터

## 기술적 특징

### GPS-SLAM 좌표계 통합
- Bag 파일의 첫 GPS 좌표를 UTM 원점으로 설정
- FasterLIO의 odom 프레임과 GPS 웨이포인트를 동일한 좌표계에서 표시
- 실시간 TF 브로드캐스트 (map ↔ odom)

### 3단계 보정 알고리즘
1. **위치 초기화**: 첫 실시간 GPS로 UTM 원점 설정
2. **초기 정렬**: 2m 이동 후 GPS 방향 기반 heading 보정
3. **점진적 보정**: 10초마다 GPS와 비교하여 10%씩 점진적 보정

### 자동화 기능
- Bag 파일 자동 재생 및 GPS 원점 추출
- 거리 필터링으로 궤적 포인트 저장 (메모리 최적화)
- 웹 브라우저 자동 실행
- 실시간 성능 분석 및 리포트 생성

## 개발 이력

### 2025년 3월
- gps_server.py로 웹 프롬프트 제공
- Waypoint 접근 시 삭제 로직 구현 (5m 이내)
- ROS 토픽 구조 설계 (/waypoints 등)
- gps_publisher.py 및 기능 보완
- 목적지 검색창 제거, waypoint ROS 토픽 전송

### 2025년 5월
- FasterLIO 연동을 통한 SLAM 기반 경로 시각화 추가
- Bag 파일 자동 재생 및 GPS 원점 추출 기능
- RViz를 통한 실시간 궤적 및 Global waypoints 시각화
- UTM 좌표 변환을 통한 GPS-SLAM 좌표계 통합
- **Waypoint 잔상 문제 해결**: MarkerArray 사용으로 이전 waypoint 마커 자동 삭제
- **Global path 업데이트 문제 해결**: 즉시 시각화 업데이트 및 타이밍 이슈 개선

### 2025년 6월
- **3단계 FasterLIO 보정 시스템 구현**
- **실시간 성능 분석기** (`analyzer.py`) 추가
- **완전한 데이터 저장 시스템** 구축 (JSON/CSV)
- **bag_player.py 독립화**


## 요구사항

- ROS Noetic
- FasterLIO
- Python 3
- 필수 패키지: `utm`, `numpy`, `matplotlib`, `pandas` (선택사항)


## 기타 사항
- /waypoints 토픽을 통해 RViz에서 시각화 가능
- GPS 수신 기반이라 야외 주행을 위한 기반 기능임

