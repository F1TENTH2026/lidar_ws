#!/bin/bash
# ============================================================
#  Hokuyo LiDAR 네트워크 설정 스크립트 (배포용)
#  - USB-Ethernet 어댑터를 통해 Hokuyo UST-10LX에 연결
#  - NetworkManager에 고정 IP 등록 (재부팅 후에도 유지)
#  - Hokuyo 기본 IP: 192.168.0.10 / Port: 10940
#
#  사용법:
#    chmod +x hokuyo_setup.sh
#    ./hokuyo_setup.sh
# ============================================================

set -e

# ── 설정값 (필요시 수정) ─────────────────────────────────────
HOST_IP="192.168.0.15"       # 이 PC에 설정할 IP
SUBNET="24"                  # 서브넷 마스크
LIDAR_IP="192.168.0.10"      # Hokuyo 기본 IP
LIDAR_PORT="10940"           # Hokuyo 기본 포트
CON_NAME="hokuyo"            # NetworkManager 연결 이름
# ─────────────────────────────────────────────────────────────

echo "======================================================"
echo "  Hokuyo LiDAR 연결 설정 시작 (배포용)"
echo "======================================================"

# ── 1. USB-Ethernet 인터페이스 자동 탐색 ──────────────────────
echo ""
echo "[1/6] USB-Ethernet 인터페이스 탐색 중..."

IFACE=$(ip -br link | awk '$1 ~ /^enx/ {print $1}' | head -n 1)

if [ -z "$IFACE" ]; then
    echo "  [오류] USB-Ethernet 인터페이스(enx...)를 찾을 수 없습니다."
    echo "         어댑터가 꽂혀 있는지 확인하세요."
    exit 1
fi

echo "  발견된 인터페이스: $IFACE"

# ── 2. 인터페이스 활성화 ──────────────────────────────────────
echo ""
echo "[2/6] 인터페이스 활성화 중..."
sudo ip link set "$IFACE" up
sleep 1

CARRIER=$(cat /sys/class/net/"$IFACE"/carrier 2>/dev/null || echo "0")
if [ "$CARRIER" != "1" ]; then
    echo "  [경고] 물리적 연결(carrier)이 감지되지 않습니다."
    echo "         LiDAR 전원 및 케이블 연결을 확인하세요."
    echo "         계속 진행하려면 Enter, 중단하려면 Ctrl+C"
    read -r
fi
echo "  인터페이스 UP 완료"

# ── 3. NetworkManager 고정 IP 등록 ────────────────────────────
echo ""
echo "[3/6] NetworkManager에 고정 IP 등록 중..."

# 기존 설정 있으면 삭제
sudo nmcli con delete "$CON_NAME" 2>/dev/null || true

# 고정 IP로 새로 등록
sudo nmcli con add type ethernet \
    ifname "$IFACE" \
    con-name "$CON_NAME" \
    ipv4.method manual \
    ipv4.addresses "$HOST_IP/$SUBNET" \
    ipv6.method disabled \
    connection.autoconnect yes

echo "  NetworkManager 등록 완료 (재부팅 후에도 유지됩니다)"

# ── 4. 연결 적용 ──────────────────────────────────────────────
echo ""
echo "[4/6] 연결 적용 중..."
sudo nmcli con up "$CON_NAME"
sleep 2
echo "  적용 완료"

# ── 5. IP 확인 ────────────────────────────────────────────────
echo ""
echo "[5/6] 네트워크 상태 확인"
ip addr show "$IFACE"

# ── 6. Hokuyo ping 테스트 ─────────────────────────────────────
echo ""
echo "[6/6] Hokuyo LiDAR ping 테스트 ($LIDAR_IP)..."
if ping -c 3 -W 2 -I "$IFACE" "$LIDAR_IP" > /dev/null 2>&1; then
    echo "  [성공] Hokuyo LiDAR에 연결되었습니다!"
else
    echo "  [실패] LiDAR에 ping이 닿지 않습니다."
    echo "         - LiDAR IP가 $LIDAR_IP 인지 확인하세요"
    echo "         - LiDAR 전원이 켜져 있는지 확인하세요"
    echo "         - 케이블 연결 상태를 확인하세요"
    exit 1
fi

# ── 완료 ──────────────────────────────────────────────────────
echo ""
echo "======================================================"
echo "  설정 완료!"
echo "  이 PC IP  : $HOST_IP"
echo "  LiDAR IP  : $LIDAR_IP:$LIDAR_PORT"
echo "  재부팅 후에도 IP가 자동으로 설정됩니다."
echo "======================================================"
echo ""
echo "  ROS2로 LiDAR 실행하려면:"
echo "  ros2 launch urg_node urg_node_launch.py sensor_interface:=ethernet"
echo ""
