#!/bin/bash
# 시연용 3인칭 추적 카메라 — Gazebo GUI가 뜬 뒤 실행하면
# 카메라가 로봇을 사람 눈높이 3인칭 구도로 따라간다.
#
# 사용법:  ./verification/demo_cam.sh [엔티티] [x] [y] [z]
#   기본값: go2, 뒤 3.0m / 옆 0.0m / 높이 1.65m(사람 눈높이)
#   예) 측면 구도:  ./verification/demo_cam.sh go2 -2.0 2.5 1.65
#
# 주의: 버전에 따라 오프셋이 로봇 회전을 따라 돌지 않고 월드 방향으로
# 고정될 수 있음 — 직선 주행 시연에는 무관.

source /opt/ros/jazzy/setup.bash 2>/dev/null

ENTITY=${1:-go2}
OX=${2:--3.0}
OY=${3:-0.0}
OZ=${4:-1.65}

echo "추적 대상: $ENTITY, 오프셋: ($OX, $OY, $OZ) — GUI 대기 중..."
for i in $(seq 1 30); do
  OK=$(gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
       --reptype gz.msgs.Boolean --timeout 2000 \
       --req "data: \"$ENTITY\"" 2>/dev/null)
  if [ "$OK" = "data: true" ]; then
    gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
      --reptype gz.msgs.Boolean --timeout 2000 \
      --req "x: $OX, y: $OY, z: $OZ" > /dev/null 2>&1
    echo "추적 카메라 적용 완료 (해제: Gazebo에서 우클릭 → Follow 해제 또는 ESC)"
    exit 0
  fi
  sleep 2
done
echo "실패: Gazebo GUI(/gui/follow 서비스)를 60초 내 찾지 못함 — gui:=true로 실행 중인지 확인"
exit 1
