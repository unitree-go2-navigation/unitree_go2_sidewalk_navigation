#!/usr/bin/env python3
"""small_city.sdf → small_city_test.sdf 축소 생성기.

sidewalk_16 밴드(x[-11.3,41.3], y[3.7,6.35])와 '선/면이 닿는' 개체만 유지.
- 인라인 model: polyline 점/box 크기로 실제 bbox 계산
- include: 카테고리별 반경 추정치로 bbox 근사
- actor/기반 요소(scene·physics·light·평면류)는 유지
텍스트 기반 블록 절제(주석·포맷 보존). --list 는 판정표만 출력.
"""
import re, sys

SRC = 'ros2_ws/src/go2_simulation/worlds/small_city.sdf'
DST = 'ros2_ws/src/go2_simulation/worlds/small_city_test.sdf'

# sidewalk_16 bbox + 접촉 허용 오차
SW = (-11.3, 3.7, 41.3, 6.35)
TOL = 0.30

# include 카테고리 → xy 반경 추정 (모델 원점 기준 절반 크기, 넉넉히)
RADIUS = {
    'house_1': 6.0, 'house_2': 6.0, 'house_3': 6.0, 'apartment': 8.0,
    'salon': 6.0, 'law_office': 6.0, 'thrift_shop': 6.0, 'fast_food': 6.0,
    'post_office': 6.0, 'osrf_first_office': 8.0, 'gas_station': 8.0,
    'oak_tree': 2.5, 'pine_tree': 2.0,
    'lamp_post': 0.5, 'telephone_pole': 0.5, 'stop_light_post': 0.6,
    'stop_sign': 0.4, 'fire_hydrant': 0.3, 'postbox': 0.5,
    'pickup': 2.7, 'suv': 2.6, 'hatchback': 2.2, 'hatchback_blue': 2.2,
    'hatchback_red': 2.2, 'ambulance': 3.0,
    'dumpster': 1.2, 'cardboard_box': 0.5, 'fountain': 3.0,
    'truss_bridge': 12.0, 'pier': 10.0, 'radio_tower': 3.0,
    'gazebo': 3.0, 'ocean': 0.0, 'city_terrain': 0.0,  # 기반: 별도 유지
}
KEEP_ALWAYS_URI = {'city_terrain'}  # 지반 하이트맵
KEEP_ALWAYS_MODEL = {'ground_plane', 'city_base_plane', 'asphalt_plane',
                     'sidewalk_16', 'sidewalk_16_raised_collision_test'}

def overlaps(bb):
    x0, y0, x1, y1 = bb
    return not (x1 < SW[0]-TOL or x0 > SW[2]+TOL or y1 < SW[1]-TOL or y0 > SW[3]+TOL)

lines = open(SRC).readlines()

# ---- 최상위 블록 스캔 (world 직속 depth) ----
blocks = []  # (kind, name, start, end)  0-index inclusive
i = 0
opener = re.compile(r'^\s*<(include|model|actor|light)(\s|>)')
while i < len(lines):
    m = opener.match(lines[i])
    if not m:
        i += 1; continue
    tag = m.group(1)
    depth = 0
    j = i
    while j < len(lines):
        depth += len(re.findall(r'<%s(?=[\s>])' % tag, lines[j]))
        depth -= len(re.findall(r'</%s>' % tag, lines[j]))
        if depth == 0:
            break
        j += 1
    body = ''.join(lines[i:j+1])
    if tag == 'include':
        nm = re.search(r'<name>([^<]+)</name>', body)
        uri = re.search(r'model://([^</]+)', body)
        name = nm.group(1) if nm else (uri.group(1) if uri else '?')
    else:
        nm = re.search(r'name="([^"]+)"', lines[i]) or re.search(r'name="([^"]+)"', body)
        name = nm.group(1) if nm else '?'
    blocks.append([tag, name, i, j, body])
    i = j + 1

# ---- 판정 ----
def include_bbox(body):
    p = re.search(r'<pose>\s*([-\d.eE\s]+)</pose>', body)
    if not p: return None
    v = [float(x) for x in p.group(1).split()[:2]]
    uri = re.search(r'model://([^</]+)', body).group(1)
    r = RADIUS.get(uri, 3.0)
    return (v[0]-r, v[1]-r, v[0]+r, v[1]+r)

def model_bbox(body):
    # 모델 pose (링크·지오메트리 오프셋은 인라인 모델들이 0 기준이라 단순 합산)
    p = re.search(r'<pose>\s*([-\d.eE\s]+)</pose>', body)
    px, py = (0.0, 0.0)
    if p:
        v = [float(x) for x in p.group(1).split()[:2]]
        px, py = v[0], v[1]
    pts = re.findall(r'<point>\s*([-\d.eE]+)\s+([-\d.eE]+)\s*</point>', body)
    if pts:
        xs = [float(a)+px for a, b in pts]; ys = [float(b)+py for a, b in pts]
        return (min(xs), min(ys), max(xs), max(ys))
    box = re.search(r'<size>\s*([-\d.eE]+)\s+([-\d.eE]+)', body)
    if box:
        hx, hy = float(box.group(1))/2, float(box.group(2))/2
        return (px-hx, py-hy, px+hx, py+hy)
    return (px-1, py-1, px+1, py+1)

decisions = []
for b in blocks:
    tag, name, s, e, body = b
    if tag == 'light':
        keep, why = True, 'light'
    elif tag == 'actor':
        keep, why = ('sidewalk16' in name), 'actor'
    elif tag == 'include':
        uri = re.search(r'model://([^</]+)', body).group(1)
        if uri in KEEP_ALWAYS_URI:
            keep, why = True, 'base'
        else:
            bb = include_bbox(body)
            keep = bb is not None and overlaps(bb)
            why = f'r={RADIUS.get(uri,3.0)} bb=({bb[0]:.1f},{bb[1]:.1f},{bb[2]:.1f},{bb[3]:.1f})' if bb else 'no-pose'
    else:  # model
        if name in KEEP_ALWAYS_MODEL:
            keep, why = True, 'base/self'
        else:
            bb = model_bbox(body)
            keep = overlaps(bb)
            why = f'bb=({bb[0]:.1f},{bb[1]:.1f},{bb[2]:.1f},{bb[3]:.1f})'
    decisions.append((keep, tag, name, s, e, why))

if '--list' in sys.argv:
    kept = [d for d in decisions if d[0]]
    dropped = [d for d in decisions if not d[0]]
    print(f'KEEP {len(kept)} / DROP {len(dropped)} (total {len(decisions)})')
    for d in kept:
        print(f'  KEEP {d[1]:7s} {d[2]:40s} {d[5]}')
    sys.exit(0)

# ---- 생성: 탈락 블록 라인 제거 ----
drop_ranges = [(d[3], d[4]) for d in decisions if not d[0]]
drop_set = set()
for s, e in drop_ranges:
    drop_set.update(range(s, e+1))
out = [ln for k, ln in enumerate(lines) if k not in drop_set]

HEADER = '''<!--
  small_city_test — small_city의 축소판 (sidewalk_16 인접 개체만 유지).
  용도: 회피 알고리즘 실험 전용 (가벼운 반복 실험).
  small_city(원본)는 최종 회피 알고리즘 완성 시 실기체 전이 전 테스트에
  사용한다 (필요에 따라 앞당겨질 수 있음).
  생성: verification/gen_test_world.py (재생성 시 수동 편집분 소실 주의)
-->
'''
# <sdf ...> 여는 줄 바로 뒤에 헤더 삽입
for k, ln in enumerate(out):
    if '<sdf' in ln:
        out.insert(k+1, HEADER)
        break
# world name 변경
out = [ln.replace('<world name="small_city"', '<world name="small_city_test"')
       for ln in out]
# RTF 0.25 캡: L1 GPU 라이다 벽시계 상한 ~2.5Hz(월드 무관, 2026-07-23 실측).
# 원본은 부하로 자연 RTF~0.5 → 심 시간 L1 ~5Hz였고 P4~P5 튜닝 전제가 그
# 조건(풀스택 실측 RTF 0.13~0.33). 경량 월드는 RTF 1.0이 되어 심 시간
# 2.5Hz로 떨어짐 → 동일 운영점(심 L1 ~8Hz) 재현.
out = [ln.replace('<real_time_factor>1</real_time_factor>',
                  '<real_time_factor>0.25</real_time_factor>')
         .replace('<real_time_update_rate>1000</real_time_update_rate>',
                  '<real_time_update_rate>250</real_time_update_rate>')
       for ln in out]
open(DST, 'w').writelines(out)
print(f'wrote {DST}: {len(out)} lines (src {len(lines)})')
