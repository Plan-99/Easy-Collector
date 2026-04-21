"""기존 dataset의 PNG를 mp4로 일괄 재인코딩.

배경: lerobot_io.py가 PNG 파일명을 'frame_NNNNNN.png' (언더스코어)로 저장했는데
lerobot의 encode_video_frames는 'frame-NNNNNN.png' (하이픈) 패턴으로 검색해서
모든 데이터 수집에서 video encoding이 silent하게 실패해왔다. 이 스크립트는:

1. images/{feature_key}/episode_NNNNNN/frame_NNNNNN.png 들을
   frame-NNNNNN.png 로 rename
2. encode_video_frames 실행해서 videos/chunk-{c}/{feature_key}/episode_NNNNNN.mp4 생성
3. 성공한 episode만 PNG 폴더 삭제

사용:
  docker exec easy_collector_service python3 -m src.backend.scripts.reencode_pngs_to_mp4 \
      --dataset 2 --fps 8 [--keep-pngs] [--dry-run]
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

_lerobot_src = Path(__file__).resolve().parent.parent / "lerobot" / "src"
if str(_lerobot_src) not in sys.path:
    sys.path.insert(0, str(_lerobot_src))

from lerobot.datasets.video_utils import encode_video_frames

DATASET_ROOT = Path("/opt/easytrainer/datasets")


def reencode_episode(
    dataset_dir: Path,
    feature_key: str,
    episode_id: int,
    chunk: int,
    fps: int,
    keep_pngs: bool,
    dry_run: bool,
) -> tuple[bool, str]:
    """한 episode의 한 sensor PNG들을 mp4로 인코딩.

    Returns:
        (success, message)
    """
    imgs_dir = dataset_dir / "images" / feature_key / f"episode_{episode_id:06d}"
    if not imgs_dir.is_dir():
        return False, f"PNG dir missing: {imgs_dir}"

    # rename frame_NNNNNN.png → frame-NNNNNN.png
    pngs = sorted(imgs_dir.glob("frame_*.png"))
    if not pngs:
        # 이미 hyphen 형식이면 OK
        hyphen_pngs = sorted(imgs_dir.glob("frame-*.png"))
        if not hyphen_pngs:
            return False, f"no PNGs in {imgs_dir}"
        pngs = hyphen_pngs
    else:
        if dry_run:
            print(f"  [DRY] would rename {len(pngs)} files in {imgs_dir}")
        else:
            for p in pngs:
                # frame_000123.png → frame-000123.png
                new_name = "frame-" + p.name.split("_", 1)[1]
                p.rename(p.parent / new_name)

    # encode
    video_path = dataset_dir / "videos" / f"chunk-{chunk:03d}" / feature_key / f"episode_{episode_id:06d}.mp4"
    if dry_run:
        print(f"  [DRY] would encode → {video_path}")
        return True, "dry-run"

    video_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        encode_video_frames(
            imgs_dir=imgs_dir,
            video_path=video_path,
            fps=fps,
            vcodec="h264",
            pix_fmt="yuv420p",
            g=2,
            crf=30,
            overwrite=True,
        )
    except Exception as e:
        return False, f"encode failed: {type(e).__name__}: {e}"

    if not video_path.exists() or video_path.stat().st_size == 0:
        return False, f"no output at {video_path}"

    if not keep_pngs:
        shutil.rmtree(imgs_dir)

    return True, f"{video_path.stat().st_size // 1024} KB"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=int, required=True)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--keep-pngs", action="store_true",
                        help="don't delete PNG dirs after successful encode")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    dataset_dir = DATASET_ROOT / str(args.dataset)
    if not dataset_dir.exists():
        raise FileNotFoundError(dataset_dir)

    info_path = dataset_dir / "meta" / "info.json"
    info = json.loads(info_path.read_text())
    chunks_size = info.get("chunks_size", 1000)

    # discover sensors and episodes
    images_root = dataset_dir / "images"
    if not images_root.exists():
        print(f"[FATAL] no images dir at {images_root}")
        sys.exit(1)

    feature_keys = sorted(p.name for p in images_root.iterdir() if p.is_dir())
    print(f"[INFO] feature keys: {feature_keys}")

    # episode ids from any sensor (assume all sensors have same episodes)
    first_sensor = images_root / feature_keys[0]
    episodes = sorted(
        int(p.name.split("_")[1]) for p in first_sensor.iterdir()
        if p.is_dir() and p.name.startswith("episode_")
    )
    print(f"[INFO] episodes: {episodes[:5]}{'...' if len(episodes)>5 else ''} (total {len(episodes)})")

    print(f"\n[INFO] dry_run={args.dry_run}, keep_pngs={args.keep_pngs}, fps={args.fps}")
    print("=" * 70)

    total = len(episodes) * len(feature_keys)
    ok = 0
    fail = 0
    for ep_id in episodes:
        chunk = ep_id // chunks_size
        for fk in feature_keys:
            success, msg = reencode_episode(
                dataset_dir, fk, ep_id, chunk, args.fps,
                keep_pngs=args.keep_pngs, dry_run=args.dry_run,
            )
            tag = "OK " if success else "ERR"
            print(f"  [{tag}] ep {ep_id:03d} {fk:40s} {msg}")
            if success:
                ok += 1
            else:
                fail += 1

    print("=" * 70)
    print(f"DONE: {ok}/{total} ok, {fail} failed")

    # Try to clean up empty images root
    if not args.keep_pngs and not args.dry_run and fail == 0:
        try:
            for fk in feature_keys:
                fk_dir = images_root / fk
                if fk_dir.exists() and not any(fk_dir.iterdir()):
                    fk_dir.rmdir()
            if not any(images_root.iterdir()):
                images_root.rmdir()
                print(f"[CLEAN] removed empty {images_root}")
        except OSError:
            pass


if __name__ == "__main__":
    main()
