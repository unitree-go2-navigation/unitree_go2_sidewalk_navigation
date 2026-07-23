#!/usr/bin/env python3
"""small_city 보도 스트립의 결정적 Nav2 맵 생성.

SLAM 불요 — 기하가 알려져 있고 /odom이 월드 좌표 초기화라(2026-07-22 계측)
map→odom identity로 쓴다. 밴드 밖 = 점유(보도 이탈의 구조적 금지),
남측 수목 침범대(x 8~11, y≤4.1)는 실측(P5-4 월드 지도)으로 마킹.
동적 장애물(보행자 등)은 로컬 코스트맵(/scan)의 몫.

사용: python3 verification/gen_nav_map.py
출력: ros2_ws/src/go2_simulation/maps/small_city_strip.{pgm,yaml}
"""

import os

RES = 0.05
X0, X1 = -12.0, 42.0
Y0, Y1 = 0.0, 10.0
BAND_Y = (3.7, 6.35)          # 주행 가능 보도 (sidewalk_polygon과 동일)
BAND_X = (-11.3, 41.3)
# 밴드 안 정적 침범물 (월드 좌표 박스): 남측 수목 캐노피 실측
INTRUSIONS = [
    (8.0, 11.0, 3.7, 4.15),   # x0, x1, y0, y1
]

W = int((X1 - X0) / RES)
H = int((Y1 - Y0) / RES)

grid = bytearray([0] * (W * H))   # 0=occupied(black) 기본 — 밴드만 해방


def mark_free(x0, x1, y0, y1):
    for gy in range(max(0, int((y0 - Y0) / RES)),
                    min(H, int((y1 - Y0) / RES))):
        row = gy * W
        for gx in range(max(0, int((x0 - X0) / RES)),
                        min(W, int((x1 - X0) / RES))):
            grid[row + gx] = 254


def mark_occ(x0, x1, y0, y1):
    for gy in range(max(0, int((y0 - Y0) / RES)),
                    min(H, int((y1 - Y0) / RES))):
        row = gy * W
        for gx in range(max(0, int((x0 - X0) / RES)),
                        min(W, int((x1 - X0) / RES))):
            grid[row + gx] = 0


mark_free(BAND_X[0], BAND_X[1], BAND_Y[0], BAND_Y[1])
for box in INTRUSIONS:
    mark_occ(*box)

out_dir = os.path.join(os.path.dirname(__file__), '..',
                       'ros2_ws', 'src', 'go2_simulation', 'maps')
os.makedirs(out_dir, exist_ok=True)
pgm = os.path.join(out_dir, 'small_city_strip.pgm')
with open(pgm, 'wb') as f:
    f.write(f'P5\n{W} {H}\n255\n'.encode())
    # PGM은 위→아래 행 순서 = 맵 y 최대→최소
    for gy in range(H - 1, -1, -1):
        f.write(bytes(grid[gy * W:(gy + 1) * W]))
with open(os.path.join(out_dir, 'small_city_strip.yaml'), 'w') as f:
    f.write(f"""image: small_city_strip.pgm
mode: trinary
resolution: {RES}
origin: [{X0}, {Y0}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")
print(f'wrote {pgm} ({W}x{H})')
