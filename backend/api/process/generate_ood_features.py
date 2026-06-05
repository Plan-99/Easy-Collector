import os
import json
import numpy as np
import torch

from ...configs.global_configs import DATASET_DIR, resolve_checkpoint_dir
from ...utils.lerobot_io import read_episode, list_episodes


# 레퍼런스 개수 상한 — kNN 거리 계산이 O(N^2) 라 너무 크면 비용/메모리 폭증. 초과 시 subsample.
OOD_MAX_REF = 5000
OOD_KNN_K = 5
# image 레퍼런스는 backbone forward 비용이 커서 프레임을 stride 로 솎아 수집.
OOD_IMG_FRAME_STRIDE = 3


def _knn_dist_sorted(feats_np, episode_ids=None):
    """(N, D) → in-distribution kNN 평균거리 분포(정렬). percentile scoring 의 기준선.

    **episode-aware**: 같은 에피소드 프레임은 시간적으로 거의 동일해서, 단순 leave-one-out
    kNN 은 "인접 프레임까지의 거리"(아주 작음)만 재서 baseline 이 과도하게 빡빡해진다. 그러면
    추론(라이브)처럼 **새 trajectory** 의 프레임은 최근접이 다른 에피소드라 거리가 커져 전부
    OOD(≈1.0)로 찍힌다. 그래서 각 프레임의 kNN 을 **자기 에피소드를 제외한** 나머지에서 구해
    "새 trajectory 가 레퍼런스에 갖는 거리" 분포를 baseline 으로 삼는다(라이브와 동일 상황).
    """
    feats = torch.from_numpy(feats_np).float()
    dists = torch.cdist(feats, feats)
    if episode_ids is not None and len(set(episode_ids.tolist())) > 1:
        eid = torch.from_numpy(np.asarray(episode_ids))
        same = eid[:, None] == eid[None, :]
        dists = dists.masked_fill(same, float('inf'))
    else:
        dists.fill_diagonal_(float('inf'))
    kk = min(OOD_KNN_K, max(1, feats.shape[0] - 1))
    knn = dists.topk(kk, largest=False).values.mean(dim=1)
    knn = knn[torch.isfinite(knn)]  # 단일 에피소드뿐인 프레임 등 inf 제거
    return np.sort(knn.numpy())


def generate_ood_features(checkpoint, policy_obj=None, task=None, task_control=None):
    """학습 데이터셋에서 OOD reference 를 추출해 ``ood_features.npz`` 로 저장.

    - **state**: ``observation.qpos`` (전 agent concat) — lerobot_io 로 직접 추출(모델 불필요).
    - **image**: 정책의 vision backbone latent — generate/추론이 동일하게 process_image →
      preprocessor → backbone(GAP) 경로를 거쳐 표현이 일치한다(ACT 정책에 한함).

    저장 키: state_features/state_dist_sorted (+ ACT 면 image_features/image_dist_sorted).
    """
    ckpt_dir = resolve_checkpoint_dir(checkpoint['id'])
    dataset_ids = list((checkpoint.get('dataset_info') or {}).keys())

    # ── image backbone 준비 (ACT 만) ────────────────────────────────────────
    policy = None
    preprocessor = None
    img_keys = []
    vision_backbone = None
    image_resolution = (224, 224)
    state_dim = None
    if policy_obj and str(policy_obj.get('type', '')).upper() == 'ACT':
        try:
            from lerobot.policies.act.modeling_act import ACTPolicy
            from ...policies.utils import process_image, make_easytrainer_processors
            from .checkpoint_test import compute_image_latent
            policy = ACTPolicy.from_pretrained(ckpt_dir)
            policy.cuda(); policy.eval()
            if getattr(getattr(policy, 'model', None), 'backbone', None) is None:
                policy = None
            else:
                preprocessor, _ = make_easytrainer_processors(
                    policy_type='ACT', cfg=policy.config, pretrained_path=ckpt_dir,
                )
                img_keys = list(policy.config.image_features)
                settings = policy_obj.get('settings') or {}
                vision_backbone = policy_obj.get('vision_backbone') or settings.get('vision_backbone') or 'resnet18'
                state_dim = policy.config.robot_state_feature.shape[0] if policy.config.robot_state_feature else None
                meta_path = os.path.join(ckpt_dir, 'train_meta.json')
                if os.path.exists(meta_path):
                    try:
                        _ir = (json.load(open(meta_path)) or {}).get('image_resolution')
                        if isinstance(_ir, (list, tuple)) and len(_ir) == 2:
                            image_resolution = (int(_ir[0]), int(_ir[1]))
                    except Exception:
                        pass
        except Exception as e:
            print(f'[OOD] image backbone init failed ({e}); state-only.')
            policy = None

    all_states = []
    all_state_epid = []
    all_img_latents = []
    all_img_epid = []
    gi = -1  # 전 데이터셋 통합 에피소드 인덱스 (episode-aware baseline 용)
    for ds_id in dataset_ids:
        ds_dir = os.path.join(DATASET_DIR, str(ds_id))
        if not os.path.isdir(ds_dir):
            continue
        try:
            episodes = list_episodes(ds_dir)
        except Exception as e:
            print(f'[OOD] list_episodes failed ds{ds_id}: {e}')
            continue
        for ep in episodes:
            ep_idx = ep.get('episode_index') if isinstance(ep, dict) else ep
            try:
                ep_data = read_episode(ds_dir, ep_idx)
            except Exception as e:
                print(f'[OOD] read_episode failed ds{ds_id} ep{ep_idx}: {e}')
                continue
            gi += 1
            # state
            sd = ep_data.get('state_data')
            if sd is not None and len(sd) > 0:
                arr = np.asarray(sd, dtype=np.float32)
                if arr.ndim == 1:
                    arr = arr.reshape(-1, 1)
                all_states.append(arr)
                all_state_epid.append(np.full(len(arr), gi, dtype=np.int32))
            # image (ACT backbone latent)
            if policy is not None and len(all_img_latents) < OOD_MAX_REF:
                images = ep_data.get('images') or {}
                T = ep_data.get('num_frames') or 0
                for t in range(0, T, OOD_IMG_FRAME_STRIDE):
                    if len(all_img_latents) >= OOD_MAX_REF:
                        break
                    try:
                        pi = {}
                        if state_dim:
                            pi['observation.state'] = torch.zeros((1, state_dim), dtype=torch.float32).cuda()
                        ok = True
                        for key in img_keys:
                            cam = key.replace('observation.images.', '')
                            frames = images.get(cam)
                            if not frames or t >= len(frames):
                                ok = False
                                break
                            im = process_image(frames[t], vision_backbone, to_cuda=True,
                                                pixel_range='01', image_resolution=image_resolution)
                            pi[key] = im.unsqueeze(0)
                        if not ok:
                            continue
                        if preprocessor is not None:
                            pi = preprocessor(pi)
                        with torch.no_grad():
                            lat = compute_image_latent(policy, pi, img_keys)
                        if lat is not None:
                            all_img_latents.append(lat.detach().cpu().float())
                            all_img_epid.append(gi)
                    except Exception as e:
                        print(f'[OOD] image latent failed ds{ds_id} ep{ep_idx} t{t}: {e}')
                        continue

    if not all_states and not all_img_latents:
        print('[OOD] nothing collected — skipping.')
        _cleanup(policy)
        return

    ood_data = {}

    # state
    if all_states:
        from collections import Counter
        target_dim = Counter(a.shape[1] for a in all_states).most_common(1)[0][0]
        keep = [i for i, a in enumerate(all_states) if a.shape[1] == target_dim]
        sf = np.concatenate([all_states[i] for i in keep], axis=0)
        sepid = np.concatenate([all_state_epid[i] for i in keep], axis=0)
        if sf.shape[0] > OOD_MAX_REF:
            sel = np.random.choice(sf.shape[0], OOD_MAX_REF, replace=False)
            sf, sepid = sf[sel], sepid[sel]
        ood_data['state_features'] = sf
        ood_data['state_dist_sorted'] = _knn_dist_sorted(sf, sepid)
        print(f'[OOD] state_features={sf.shape}, dist=[{ood_data["state_dist_sorted"][0]:.4f}, {ood_data["state_dist_sorted"][-1]:.4f}]')

    # image
    if all_img_latents:
        imf = torch.cat(all_img_latents, dim=0).numpy()
        iepid = np.asarray(all_img_epid, dtype=np.int32)
        ood_data['image_features'] = imf
        ood_data['image_dist_sorted'] = _knn_dist_sorted(imf, iepid)
        print(f'[OOD] image_features={imf.shape}, dist=[{ood_data["image_dist_sorted"][0]:.4f}, {ood_data["image_dist_sorted"][-1]:.4f}]')

    np.savez(os.path.join(ckpt_dir, 'ood_features.npz'), **ood_data)
    print(f'[OOD] saved {list(ood_data.keys())} → {ckpt_dir}/ood_features.npz')
    _cleanup(policy)


def _cleanup(policy):
    try:
        if policy is not None:
            del policy
        import gc
        gc.collect()
        torch.cuda.empty_cache()
    except Exception:
        pass
