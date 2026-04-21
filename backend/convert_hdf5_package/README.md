# HDF5 -> LeRobot 변환 스크립트

HDF5 데이터를 LeRobot dataset v2.1 포맷으로 변환하는 스크립트입니다.

## 폴더 구조

```
convert_hdf5_package/
├── convert_from_hdf5_1piper.py
├── fast_lerobot_dataset.py
├── requirements.txt
├── README.md
└── datasets/          <- 여기에 HDF5 파일을 넣으세요
    ├── episode_0.hdf5
    ├── episode_1.hdf5
    ├── episode_2.hdf5
    └── ...
```

## 설치

```bash
pip install -r requirements.txt
```

## 실행

```bash
python convert_from_hdf5_1piper.py \
  --data-path /path/to/hdf5_dir \
  --out-dir /path/to/output \
  --task "pick up the blue object"
```

### 기본값

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--task` | `"I miss old Kanye"` | 태스크 설명 문자열 |
| `--data-path` | `./datasets` | HDF5 파일이 있는 폴더 경로 |
| `--out-dir` | `./output` | 변환 결과 출력 경로 |

경로를 생략하면 스크립트 위치 기준 `datasets/`에서 읽고 `output/`에 저장합니다.

```bash
# 기본 경로 사용 예시
python convert_from_hdf5_1piper.py --task "pick up the blue object"
```

## HDF5 파일 포맷

스크립트는 아래 구조의 HDF5 파일을 기대합니다:

```
episode_X.hdf5
├── observations/
│   ├── qpos/robot_4          # (T, 7) float32 - 관절 위치
│   └── images/
│       ├── sensor_9          # (T, H, W, 3) - cam_high
│       ├── sensor_10         # (T, H, W, 3) - cam_left_wrist
│       └── sensor_8          # (T, H, W, 3) - cam_right_wrist
└── qaction/robot_4           # (T, 7) float32 - 액션
```

## 카메라 매핑

| HDF5 키 | LeRobot 키 |
|---------|-----------|
| sensor_9 | cam_high |
| sensor_10 | cam_left_wrist |
| sensor_8 | cam_right_wrist |
