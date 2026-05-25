# 정적 / 동적 카메라 구분

- **결정일**: 2026-05-08 (SAM3 모듈 설계 논의 중 명시)
- **상태**: 정책
- **영향 폴더**: [backend/](../../backend/), [modules/extensions/sam3/](../../modules/extensions/sam3/), [frontend/](../../frontend/)

## 결정

이미지 처리 기능은 **카메라 타입**에 따라 적용 가능 여부가 다르다.

| 카메라 타입 | 예시 | 가정 |
|------------|------|------|
| **정적 카메라** | 천장캠, 작업대 고정캠 | 한번 잡은 화각·crop이 작업 내내 유효 |
| **동적 카메라** | 헤드캠, 손목캠 | 매 프레임 화각이 달라짐 |

| 기능 유형 | 정적 카메라 | 동적 카메라 |
|----------|------------|------------|
| Crop / ROI fix (frame-static) | OK | **금지** |
| Segmentation / Detection (object-static) | OK | OK |

## 배경 (Why)

`task.sensor_cropped_area` 같은 crop 설정은 frame 좌표 기반이라, 카메라가 움직이면
잘못된 영역을 잘라낸다. 반면 SAM3 같은 객체 단위 세그멘테이션은 객체를 추적하므로
양쪽 모두에서 동작한다.

> 사용자(hcjung) 원문 인용:
> "auto crop은 빼주는 게 맞아. crop은 동적인 카메라가 아니라 정적카메라에서 쓰는 기능이야.
> 세그멘테이션은 동적 카메라와 정적 카메라 모두에서 쓸 수 있어야 해."

## 적용 원칙

1. 새 이미지 처리 기능 설계 시 **"동적 카메라에서도 의미 있는가?"** 부터 확인
2. bbox 기반 auto-crop처럼 **객체 위치를 frame 중심으로 강제 이동시키는 변형**은
   두 카메라 타입 **모두에 부적합** → 제안하지 말 것
3. frame-static 가정 기능(crop, ROI fix)과 object-static 가정 기능(segmentation, detection)을
   섞어서 한 API로 묶지 말 것

## 영향 받는 곳

- `task.sensor_cropped_area` 관련 backend 로직
- SAM3 모듈 (`modules/extensions/sam3/`)
- TrainPage / SensorPage의 이미지 처리 UI
