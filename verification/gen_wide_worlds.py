#!/usr/bin/env python3
"""small_city_test.sdf → 보도 광폭 변형 월드 생성기.

- small_city_test_2x.sdf: 보도 폭 6.0m (2배) — 용도 추후 결정
- small_city_test_3x.sdf: 보도 폭 9.0m (3배) — 다수 장애물·다수 액터를
  시나리오로 배치해 Go2의 틈 사이 회피·군중 회피를 검증하는 무대

확장은 남쪽(도로 방향) — 북측은 상가 건물. 폴리라인은 북측 가장자리
y=6.7 기준 남쪽으로 선형 스케일(모서리 라운드도 비례 유지), 충돌
슬래브는 치수·중심 재계산. 새 밴드 안에 들어오는 기존 노변 물체
(주차차량·가로등·전신주·우체통·신호등·정지표지)와 액터는 제거 —
장애물·액터는 시나리오에서 명시적으로 배치한다 (빈 무대 원칙).

주의: 인지 밴드(sidewalk_polygon)·nav 점유 맵은 아직 3m 기준 —
시나리오 통합 시 월드별 밴드/맵 설정과 함께 조정할 것.
"""
import re

SRC = 'ros2_ws/src/go2_simulation/worlds/small_city_test.sdf'
NORTH = 6.7          # 보도 북측 가장자리 (고정 기준)
BASE_W = 3.0

# 새 밴드에 들어와 무대를 오염시키는 개체 (이름 기준 제거)
CLEAR_NAMES = ['pedestrian_on_sidewalk16', 'hatchback_blue_252',
               'lamp_post_192', 'lamp_post_194',
               'telephone_pole_231', 'telephone_pole_232',
               'postbox_92', 'stop_light_post_176', 'stop_light_post_178',
               'stop_sign_185']

HEADERS = {
    2: '''<!--
  small_city_test_2x — 보도 폭 6.0m (표준 3.0m의 2배, 남측 확장).
  용도: 추후 결정 (2026-07-29 기준 미정 — 사용자 지정 예정).
  월드 3종 사용 의도:
    - small_city_test      (3m): 표준 반복 실험 (좁은 보도 스트레스 조건)
    - small_city_test_2x   (6m): 용도 추후 결정
    - small_city_test_3x   (9m): 다수 장애물·군중 액터 시나리오 무대
    - small_city (원본 도시): 최종 알고리즘 완성 후 실기체 전이 전 테스트
  노변 물체·액터는 비워둠 — 시나리오에서 명시적으로 배치.
  생성: verification/gen_wide_worlds.py (small_city_test.sdf 기반)
-->
''',
    3: '''<!--
  small_city_test_3x — 보도 폭 9.0m (표준 3.0m의 3배, 남측 확장).
  용도: 다수 정적 장애물 + 다수 보행 액터를 시나리오로 배치해 Go2가
  틈 사이를 누비며 회피·교행하는 능력을 검증하는 무대.
  월드 3종 사용 의도:
    - small_city_test      (3m): 표준 반복 실험 (좁은 보도 스트레스 조건)
    - small_city_test_2x   (6m): 용도 추후 결정
    - small_city_test_3x   (9m): 다수 장애물·군중 액터 시나리오 무대
    - small_city (원본 도시): 최종 알고리즘 완성 후 실기체 전이 전 테스트
  노변 물체·액터는 비워둠 — 시나리오에서 명시적으로 배치.
  생성: verification/gen_wide_worlds.py (small_city_test.sdf 기반)
-->
''',
}


def drop_block(txt, tag, name):
    """이름이 name인 최상위 tag 블록 제거 (include는 <name>, 그 외 name=)."""
    if tag == 'include':
        pat = re.compile(
            r'<include>(?:(?!</include>).)*?<name>' + re.escape(name)
            + r'</name>.*?</include>', re.S)
    else:
        pat = re.compile(
            r'<%s name="%s".*?</%s>' % (tag, re.escape(name), tag), re.S)
    return pat.sub('', txt)


def widen(scale):
    w = BASE_W * scale
    txt = open(SRC).read()

    # 1) 기존 헤더 주석 → 변형 전용 헤더
    txt = re.sub(r'<!--.*?-->\n', HEADERS[scale], txt, count=1, flags=re.S)

    # 2) sidewalk_16 폴리라인: 북측 y=6.7 기준 남쪽으로 스케일
    m = re.search(r'(<model name="sidewalk_16">.*?</model>)', txt, re.S)
    block = m.group(1)

    def scale_pt(pm):
        x, y = float(pm.group(1)), float(pm.group(2))
        y2 = NORTH - (NORTH - y) * scale
        return f'<point>{x} {round(y2, 6)}</point>'
    block2 = re.sub(
        r'<point>\s*([-\d.eE]+)\s+([-\d.eE]+)\s*</point>', scale_pt, block)
    txt = txt.replace(block, block2)

    # 3) 충돌 슬래브: 폭 w, 중심 y = 6.7 - w/2
    txt = txt.replace(
        '<pose>15.000000 5.200000 0.160000 0 0 0</pose>',
        f'<pose>15.000000 {round(NORTH - w / 2, 3)} 0.160000 0 0 0</pose>')
    txt = txt.replace(
        '<size>52.600000 3.000000 0.01</size>',
        f'<size>52.600000 {w} 0.01</size>')

    # 4) 무대 비우기: 액터 + 새 밴드 안 노변 물체
    txt = drop_block(txt, 'actor', 'pedestrian_on_sidewalk16')
    for nm in CLEAR_NAMES[1:]:
        txt = drop_block(txt, 'include', nm)

    dst = f'ros2_ws/src/go2_simulation/worlds/small_city_test_{scale}x.sdf'
    open(dst, 'w').write(txt)
    ys = [float(b) for a, b in re.findall(
        r'<point>\s*([-\d.eE]+)\s+([-\d.eE]+)', re.search(
            r'<model name="sidewalk_16">.*?</polyline>', txt, re.S).group(0))]
    print(f'{dst}: 폴리라인 y {min(ys):.1f}~{max(ys):.1f} '
          f'({max(ys)-min(ys):.1f}m), actor={"<actor" in txt}, '
          f'lines={txt.count(chr(10))}')


for s in (2, 3):
    widen(s)
