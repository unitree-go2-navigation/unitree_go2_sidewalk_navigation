# Perception 노드 인터페이스 계약

팀원의 detection / segmentation / depth estimation 노드가 회피 스택과 통합되기 위한
토픽·메시지 계약. **이 문서가 정본이며, 계약 변경은 회피 스택 담당자와 합의 후 반영한다.**

설계 원칙:
- **표준 메시지만 사용** (`vision_msgs`, `geometry_msgs`, `sensor_msgs`) — 커스텀 msg 패키지 없음
- **카메라 스펙 비의존**: intrinsics는 `CameraInfo`로 전달, 알고리즘에 hardcode 금지
- **안전 불변식**: perception 결과는 안전 임계를 *보수화*하는 데만 쓰인다. 완화 금지.
  (class 오인식이 안전거리를 줄이는 일이 구조적으로 불가능해야 함)

---

## 1. Detection (2D 객체 검출 — YOLO 등)

| 항목 | 값 |
|---|---|
| 토픽 | `/perception/detections_2d` |
| 메시지 | `vision_msgs/Detection2DArray` |
| frame | 카메라 optical frame (`header.frame_id`에 명시) |
| 주기 | ≥ 10Hz 권장 |
| 동반 조건 | 같은 카메라의 `sensor_msgs/CameraInfo` 토픽 필수 (bbox→ray 변환용) |

규약:
- `results[0].hypothesis.class_id`는 아래 **고정 라벨맵** 문자열만 사용:
  `person`, `bicycle`, `scooter`, `dog`, `vehicle`, `other`
- `results[0].hypothesis.score` 필수 (0~1). score < 0.5는 fusion에서 무시될 수 있음.
- `header.stamp`는 **원본 이미지의 stamp**를 그대로 사용 (fusion이
  `ApproximateTimeSynchronizer`, max_time_diff 0.1s로 LiDAR와 동기화).
- 소비자: `class_fusion_node` (Track V) — LiDAR 클러스터에 angular 매칭으로 class 부여.
  class는 임계 보수화에만 사용됨 (예: bicycle → 더 긴 stop 거리).

## 2. Segmentation (인도 경계)

| 항목 | 값 |
|---|---|
| 토픽 | `/perception/sidewalk/boundary` |
| 메시지 | `geometry_msgs/PolygonStamped` |
| frame | `base_link` |
| 주기 | ≥ 5Hz |
| 범위 | 로봇 전방 ~8m의 주행 가능(인도) 영역 폴리곤 |

규약:
- **mask → 지면 투영 → 폴리곤 근사는 segmentation 노드가 소유**한다.
  (소비자마다 intrinsics/지면 투영을 다시 구현하지 않기 위함 — 카메라 비의존 원칙)
- 디버그용 raw mask는 `/perception/sidewalk/mask` (`sensor_msgs/Image`, mono8)로 별도 발행 가능.
- 폴리곤 꼭짓점 순서: 반시계(CCW). 자기교차 금지. 꼭짓점 수 ≤ 64 권장.
- **Timeout 정책 (소비자 측)**: 1.0s 이상 미수신 시 정적 YAML 폴리곤
  (`sidewalk_boundaries.yaml`)으로 폴백, 그것도 없으면 lateral/YIELD 비활성(STOP-only 모드).
- 소비자: `sidewalk_polygon_node`/YIELD 로직 (Phase 5) — 시뮬 단계에서는 정적 YAML이
  같은 토픽으로 발행되므로, 실기체 전환 시 **remap만으로 스왑**된다.

## 3. Depth Estimation

| 항목 | 값 |
|---|---|
| 토픽 | `/perception/depth` |
| 메시지 | `sensor_msgs/Image`, encoding `32FC1`, 단위 meter |
| frame | 카메라 optical frame |
| 동반 조건 | 대응 `CameraInfo` 필수 |

규약:
- D435i의 `/d435i/depth_image`와 **drop-in 호환** (같은 인코딩/단위) — 소비자는
  launch remap만으로 하드웨어 depth ↔ 추정 depth를 교체할 수 있어야 한다.
- 무효 픽셀은 `NaN` 또는 `0.0` (소비자는 둘 다 무효로 처리).
- **주의**: 안전 거리/TTC는 LiDAR 단독 원칙 유지. depth는 발 근처 장애물(Phase 6)과
  분류 보조에만 사용.

---

## 통합 테스트

- `verification/`에 각 계약의 스텁 퍼블리셔 + 계약 준수 체크 스크립트 제공 예정 (Phase 5 진입 시).
- 팀원 노드는 스텁과 동일 토픽/메시지로 교체 가능해야 하며, 통합 전
  `ros2 topic info -v`로 QoS·타입 확인, stamp가 원본 센서 시각인지 확인할 것.

## TF 요구사항

- 카메라 노드는 `base_link → <camera_optical_frame>` TF가 tree에 존재해야 함
  (URDF 또는 static_transform_publisher). 실기체에서는 캘리브레이션 값으로 갱신.
