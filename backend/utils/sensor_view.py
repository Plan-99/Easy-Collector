# -*- coding: utf-8 -*-
"""Per-view sensor key helpers.

EasyTrainer 의 워크스페이스/태스크는 같은 물리 센서를 여러 번 포함할 수 있다
(각각 다른 crop/rotate/resize → 한 카메라의 여러 ROI 를 별도 채널처럼 다룸).
이 모듈은 `sensor_ids: list[int]` (중복 허용) 을 view 단위 키로 펼치는 한
가지 정해진 방법을 제공한다.

### 키 규칙

같은 sensor_id 가 ``sensor_ids`` 안에서 **N 번째 (0-based)** 로 나타나면:

    N == 0       →  view_key = str(sensor_id)            예: "5"
    N >= 1       →  view_key = f"{sensor_id}_{N+1}"      예: "5_2", "5_3"

첫 view 가 suffix 없이 그냥 sensor_id 인 이유는, single-view (모든 sensor 가
한 번씩만 등장) 환경의 기존 데이터셋/체크포인트와 **bit-for-bit 동일** 한
키를 유지하기 위함이다. 이 덕분에 마이그레이션 스크립트 없이 양방향 호환된다.

### 사용 패턴

전체 task 의 view 들을 한꺼번에 뽑을 때::

    for sensor_id, vkey in enumerate_views(task["sensor_ids"]):
        crop = task["sensor_cropped_area"].get(vkey)
        ...

가끔 sensor_id 만 알고 occurrence 를 직접 지정해야 하는 경우 (예: 마이그레이션
스크립트, 디버그 로그) 는 ``view_key`` 를 직접 호출.
"""

from __future__ import annotations

from typing import Iterable, List, Tuple


def view_key(sensor_id: int | str, occurrence_index: int) -> str:
    """sensor_id + occurrence index 를 view_key 로.

    Args:
        sensor_id: 물리 센서 ID (int 또는 int-string).
        occurrence_index: 같은 sensor_id 가 ``sensor_ids`` 에서 몇 번째 (0-based).

    Returns:
        view_key 문자열. occurrence_index=0 이면 suffix 없음.

    >>> view_key(5, 0)
    '5'
    >>> view_key(5, 1)
    '5_2'
    >>> view_key(7, 2)
    '7_3'
    """
    if occurrence_index < 0:
        raise ValueError(f"occurrence_index must be >= 0, got {occurrence_index}")
    sid = str(sensor_id)
    if occurrence_index == 0:
        return sid
    return f"{sid}_{occurrence_index + 1}"


def enumerate_views(sensor_ids: Iterable) -> List[Tuple[int, str]]:
    """``sensor_ids`` 리스트를 [(sensor_id, view_key), ...] 로 펼침.

    같은 sensor_id 가 여러 번 나오면 첫 occurrence 는 suffix 없는 키, 이후엔
    ``_2``, ``_3`` ... 가 붙음. occurrence 는 **입력 순서** 로 계산하고,
    결과는 **(sensor_id, occurrence) 오름차순으로 정렬** 한다. 같은 sensor
    의 view 들이 항상 인접해서 묶이도록 — MonitoringWindow viewport / 데이터셋
    feature 순서 / 학습 dataloader / 추론 image_features 모두 같은 ordering
    을 따른다.

    >>> enumerate_views([5, 7])
    [(5, '5'), (7, '7')]
    >>> enumerate_views([5, 7, 5])
    [(5, '5'), (5, '5_2'), (7, '7')]
    >>> enumerate_views([7, 5, 7, 5])
    [(5, '5'), (5, '5_2'), (7, '7'), (7, '7_2')]
    >>> enumerate_views([5, 5, 5])
    [(5, '5'), (5, '5_2'), (5, '5_3')]
    """
    # Pass 1: occurrence 계산 — 입력 순서 보존 (어떤 view 가 occurrence 0 인지는
    # "먼저 들어온 것" 이 기준이므로).
    seen: dict[int, int] = {}
    items: List[Tuple[int, int, str]] = []
    for sid in sensor_ids:
        sid_int = int(sid)
        idx = seen.get(sid_int, 0)
        items.append((sid_int, idx, view_key(sid_int, idx)))
        seen[sid_int] = idx + 1
    # Pass 2: (sensor_id, occurrence) 로 정렬 — 같은 sensor 의 view 들이 인접.
    items.sort(key=lambda t: (t[0], t[1]))
    return [(sid, vkey) for sid, _occ, vkey in items]


def parse_view_key(vkey: str) -> Tuple[int, int]:
    """view_key 를 (sensor_id, occurrence_index) 로 역변환.

    저장된 데이터 (dataset feature 이름 등) 에서 sensor_id 를 뽑아야 할 때.

    >>> parse_view_key("5")
    (5, 0)
    >>> parse_view_key("5_2")
    (5, 1)
    >>> parse_view_key("7_3")
    (7, 2)
    """
    if "_" not in vkey:
        return int(vkey), 0
    head, _, tail = vkey.rpartition("_")
    # tail 이 정수가 아닐 수도 있음 (예: 비정형 key). 그러면 단일 view 로 간주.
    try:
        n = int(tail)
    except ValueError:
        return int(vkey), 0
    if n < 2:
        # "5_1" 같이 suffix 가 1 인 경우는 우리 규칙상 발생 안 함 — 그래도 방어적으로
        # occurrence 0 으로 해석 (suffix 없는 게 표준).
        return int(head), 0
    return int(head), n - 1


def is_same_sensor(vkey_a: str, vkey_b: str) -> bool:
    """두 view_key 가 같은 물리 센서를 가리키는지.

    >>> is_same_sensor("5", "5_2")
    True
    >>> is_same_sensor("5", "7")
    False
    """
    try:
        sa, _ = parse_view_key(vkey_a)
        sb, _ = parse_view_key(vkey_b)
    except ValueError:
        return False
    return sa == sb
