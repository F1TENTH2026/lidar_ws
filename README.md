# F1Tenth LiDAR Workspace

Hokuyo UST-10LX LiDAR 드라이버 + DBSCAN 클러스터링 + RViz2 시각화

---

## 1. 최초 설정 (한 번만)

```bash
# 네트워크 설정 (USB-Ethernet 어댑터 → LiDAR)
# 자신의 워크스페이스로 이동 필수 
chmod +x hokuyo_setup.sh
./hokuyo_setup.sh

# 의존성 빌드
colcon build --symlink-install
```

---

## 2. 실행

```bash
source install/setup.bash
ros2 launch f1tenth_lidar lidar_with_clustering.py
```

> LiDAR 드라이버 + 클러스터링 노드 + RViz2 가 한 번에 실행됩니다.

---

## 3. 주요 파라미터 변경

```bash
# Serial 연결
ros2 launch f1tenth_lidar lidar_with_clustering.py sensor_interface:=serial

# 클러스터링 민감도 조정
ros2 launch f1tenth_lidar lidar_with_clustering.py \
    cluster_tolerance:=0.2 \
    min_cluster_size:=5 \
    max_range:=8.0

# RViz2 없이 실행
ros2 launch f1tenth_lidar lidar_with_clustering.py rviz:=false
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `sensor_interface` | `ethernet` | 연결 방식: `ethernet` / `serial` |
| `cluster_tolerance` | `0.15` | 클러스터 묶는 거리 (m) |
| `min_cluster_size` | `3` | 클러스터 최소 포인트 수 |
| `max_range` | `10.0` | 유효 감지 거리 (m) |

---

## 4. 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 원시 데이터 |
| `/lidar_clusters` | `visualization_msgs/MarkerArray` | 클러스터 시각화 |

---

## 5. LiDAR 연결 확인

```bash
ping 192.168.0.10
```
