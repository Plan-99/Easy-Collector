# -*- coding: utf-8 -*-
"""
End-to-end Easy Trainer pipeline smoke test.

이 테스트는 시드된 tutorial 환경(tutorial_arm + 두 개 카메라 + tutorial_agent
assembly + tutorial_env workspace) 위에서 다음 플로우를 단일 파이프라인으로
실행해 어디에서도 끊김/예외가 없는지 검증한다:

    1. backend / training_server / mujoco sim 헬스 체크
    2. tutorial 시드(_ensure_tutorial_rows)가 만든 robot/sensor/agent/workspace
       조회 + 부족하면 시드를 한 번 더 트리거
    3. tutorial sim 기동 + 토픽 활성 대기
    4. 로봇 subscribe → current_app.agents 등록
    5. 데이터셋 생성 + tele_type='keyboard' 로 record 시작
    6. socketio 로 move_robot_joint_delta / move_robot_ee_delta 이벤트를
       주입해 키보드 텔리옵 입력을 모방, 두 에피소드를 마저 저장
    7. --action-types 에 지정된 각 action_type(기본: qaction, relative_ee_pos)
       마다 7a~7e 단계를 반복:
       7a. ACT policy 행 생성(action_type 지정) + checkpoint 생성
       7b. /train/queue enqueue → 로컬 training_server(localhost:5100)에서 학습
       7c. status='finished' 까지 polling
       7d. /checkpoint/<id>/:start_test 로 추론 짧게 돌리고 정상 stop
       7e. planner 생성 — joint_position(homepose) → timesleep → checkpoint(추론)
           3블록 plan 만들고 :start_run, planner_run_end='finished' 까지 대기

action_type 별 차이는 policy.settings 의 action_type 한 값뿐이다. 같은 녹화
데이터셋(observation.eepos 포함 — tutorial_arm 은 ik_setting 이 있어 record 시
EE pose 가 같이 저장됨)을 두 action_type 학습에 그대로 재사용한다.
'relative_ee_pos' 는 EE local-frame chunk trajectory, 'qaction' 은 절대 joint
position target 으로 학습/추론한다.

이 스크립트는 backend(host network 5000) + training_server(5100) + ROS2 bridge
가 이미 떠 있다고 가정한다. 실패 시 어느 단계에서 어떤 사유로 실패했는지를
표준 출력에 한 줄씩 stage 형태로 남긴다.

Usage:
    python -m backend.tests.test_e2e_pipeline                # default (qaction + relative_ee_pos)
    python -m backend.tests.test_e2e_pipeline --action-types qaction
    python -m backend.tests.test_e2e_pipeline --action-types relative_ee_pos
    python -m backend.tests.test_e2e_pipeline --hz 10 --episodes 2 --train-epochs 100
    python -m backend.tests.test_e2e_pipeline --skip-train   # collection만
"""
import argparse
import json
import sys
import threading
import time
from contextlib import contextmanager

import requests

try:
    import socketio
except ImportError:  # pragma: no cover
    print('[FATAL] python-socketio[client] is required: pip install "python-socketio[client]"')
    raise


DEFAULT_BACKEND = 'http://127.0.0.1:5000'
DEFAULT_TRAINING_SERVER = 'http://127.0.0.1:5100'


# ---------------------------------------------------------------------------
# Stage / log helpers
# ---------------------------------------------------------------------------

class StageError(RuntimeError):
    """단계 실행 중 발생한 도메인 오류 — 상위에서 catch 해 한 줄 요약을 남긴다."""


@contextmanager
def stage(name):
    print(f'\n=== [STAGE] {name} ...', flush=True)
    t0 = time.time()
    try:
        yield
    except StageError as e:
        print(f'[FAIL] {name}: {e}', flush=True)
        raise SystemExit(1)
    except Exception as e:  # noqa: BLE001 — 단계별 자유 예외도 한번에 포착
        import traceback
        traceback.print_exc()
        print(f'[FAIL] {name}: {e}', flush=True)
        raise SystemExit(1)
    finally:
        print(f'=== [STAGE done in {time.time() - t0:.1f}s] {name}', flush=True)


# ---------------------------------------------------------------------------
# Backend HTTP helpers
# ---------------------------------------------------------------------------

class Backend:
    def __init__(self, base_url):
        self.base_url = base_url.rstrip('/')

    def _url(self, path):
        if path.startswith('/'):
            return f'{self.base_url}/api{path}'
        return f'{self.base_url}/api/{path}'

    def get(self, path, **kwargs):
        return self._call('GET', path, **kwargs)

    def post(self, path, **kwargs):
        return self._call('POST', path, **kwargs)

    def put(self, path, **kwargs):
        return self._call('PUT', path, **kwargs)

    def delete(self, path, **kwargs):
        return self._call('DELETE', path, **kwargs)

    def _call(self, method, path, **kwargs):
        url = self._url(path)
        timeout = kwargs.pop('timeout', 30)
        try:
            resp = requests.request(method, url, timeout=timeout, **kwargs)
        except requests.exceptions.RequestException as e:
            raise StageError(f'{method} {url} failed: {e}') from e
        if resp.status_code >= 400:
            raise StageError(f'{method} {url} -> HTTP {resp.status_code}: {resp.text[:200]}')
        try:
            return resp.json()
        except ValueError:
            return {'raw': resp.text}


# ---------------------------------------------------------------------------
# Tutorial setup
# ---------------------------------------------------------------------------

def ensure_tutorial_running(api: Backend, max_wait_topics=60.0, show_viewer=None):
    """tutorial 시드 행을 강제로 조회/생성 + sim 을 켠다.

    tutorial:start 응답이 robot_id, sensor_ids, assembly_id, workspace_id 를
    모두 돌려주므로(이번 패치에서 추가) 그 값을 그대로 들고 후속 단계에
    전달한다. 이미 떠 있어도 idempotent — bridge driver의 StartLaunch가
    같은 process_id면 기존을 stop 후 재시작하므로 show_viewer 옵션도 같이
    바뀐다.

    show_viewer:
        None  → backend 기본값(True) 유지
        False → headless로 기동 (viewer 창 안 열림, 카메라는 EGL offscreen)
        True  → 명시적으로 viewer 창 열기
    """
    body = {}
    if show_viewer is not None:
        body['show_viewer'] = bool(show_viewer)
    res = api.post('/tutorial:start', json=body)
    if res.get('status') != 'success':
        raise StageError(f'tutorial:start failed: {res}')

    required = ('robot_id', 'sensor_ids', 'assembly_id', 'workspace_id')
    for key in required:
        if res.get(key) in (None, []):
            raise StageError(f'tutorial:start did not return {key}: {res}')

    # 토픽이 올라올 때까지 잠시 대기 (mujoco 부팅이 느릴 수 있음).
    # `running` 은 bridge driver의 process list 추적 상태일 뿐 — 이전 테스트가
    # sim 을 켜둔 채 끝났거나 driver가 재시작되면 has_topics=True 인데
    # running=False 인 상태가 정상이다. 실제 record/inference 가능 여부는
    # has_topics 한 가지로 충분.
    deadline = time.time() + max_wait_topics
    last_status = None
    while time.time() < deadline:
        last_status = api.get('/tutorial/status')
        if last_status.get('has_topics'):
            print(f'  tutorial sim ready: robot_id={res["robot_id"]} '
                  f'sensors={res["sensor_ids"]} assembly={res["assembly_id"]} '
                  f'workspace={res["workspace_id"]} '
                  f'(driver_running={last_status.get("running")})')
            return res
        time.sleep(1.0)
    raise StageError(f'tutorial topics did not appear within {max_wait_topics}s: {last_status}')


def subscribe_robot(api: Backend, robot_id):
    """current_app.agents 에 RemoteAgent를 등록하고 토픽 구독을 시작.

    이 호출 없이 record/start_test 를 부르면 KeyError(agents[robot_id])가 난다.
    """
    res = api.post(f'/robot/{robot_id}/:subscribe_robot')
    if res.get('status') != 'success':
        raise StageError(f'subscribe_robot failed: {res}')


def start_robot_driver(api: Backend, robot):
    """custom 로봇의 robot:start 호출. custom 은 외부 토픽만 소비하므로 단순
    success 응답으로 no-op (built-in robot 만 driver 가 실제로 뜬다)."""
    payload = {
        'id': robot['id'],
        'type': robot['type'],
        'company': robot.get('company') or '',
        'process_id': f'robot_{robot["id"]}',
        'settings': {},  # backend 가 DB row 에서 다시 hydrate
    }
    res = api.post('/robot:start', json=payload)
    if res.get('status') != 'success':
        raise StageError(f'/robot:start failed: {res}')


def fetch_workspace_payload(api: Backend, workspace_id):
    """record_episode 가 기대하는 task/robots/sensors 를 한 번에 묶어서 돌려준다."""
    res = api.get(f'/tasks/{workspace_id}')
    if res.get('status') != 'success':
        raise StageError(f'/tasks/{workspace_id} failed: {res}')
    task = res['task']
    if not task.get('assembly') or not task['assembly'].get('robots'):
        raise StageError(f'workspace {workspace_id} has no assembly/robots: {task}')
    if not task.get('sensors'):
        raise StageError(f'workspace {workspace_id} has no sensors: {task}')
    return task


def _list_tasks(api: Backend):
    """GET /tasks를 호출해 task 리스트만 평탄화해 돌려준다."""
    res = api.get('/tasks')
    tasks = res.get('tasks', res) if isinstance(res, dict) else res
    if isinstance(tasks, dict):
        tasks = tasks.get('tasks', [])
    return tasks or []


def cleanup_test_workspaces(api: Backend, prefix='test_by_code_'):
    """`prefix`로 시작하는 모든 워크스페이스(=task)를 삭제하고 삭제된 id 목록을 반환."""
    deleted = []
    for t in _list_tasks(api):
        name = t.get('name') or ''
        if name.startswith(prefix):
            tid = t.get('id')
            try:
                api.delete(f'/task/{tid}')
                deleted.append(tid)
            except StageError as e:
                print(f'  [WARN] failed to delete task {tid}: {e}', flush=True)
    return deleted


def clone_workspace(api: Backend, source_id, new_name):
    """`source_id` 워크스페이스의 설정(assembly/sensors/home_pose/settings 등)을
    그대로 복제한 새 워크스페이스를 만들고 새 id를 반환한다.

    POST /api/task 응답이 id를 돌려주지 않으므로 GET /tasks에서 같은 이름으로
    매칭해 새 id를 찾는다. 같은 이름이 여러 개면 가장 큰 id(가장 최근 생성)를
    선택.
    """
    # 1) 원본 워크스페이스 데이터 가져오기
    src = fetch_workspace_payload(api, source_id)

    # 2) 새 워크스페이스 생성 (이름만 넣어 빈 row 만들기)
    api.post('/task', json={'name': new_name})

    # 3) 새 id 매칭
    matched = [t['id'] for t in _list_tasks(api)
               if (t.get('name') or '') == new_name]
    if not matched:
        raise StageError(f'failed to find newly created workspace {new_name!r}')
    new_id = max(matched)

    # 4) 원본의 필드 복사 (PUT)
    payload = {
        'assembly_id': src.get('assembly_id'),
        'home_pose': src.get('home_pose'),
        'image': src.get('image'),
        'episode_len': src.get('episode_len'),
        'sensor_ids': src.get('sensor_ids'),
        'settings': src.get('settings'),
    }
    api.put(f'/task/{new_id}', json=payload)
    return new_id


# ---------------------------------------------------------------------------
# Socket.IO helper — 키보드 텔리옵 입력 모방
# ---------------------------------------------------------------------------

class KeyboardInjector:
    """별도 스레드에서 socketio로 move_robot_joint 이벤트를 일정 주기마다
    쏴서 키보드 누름을 모방한다. record_episode(tele_type='keyboard') 루프가
    `agent.moved_by_ui` flag 가 True 가 되어야 한 step 진행하므로, hz보다 더
    자주 emit 해 두면 안전하다.

    이전 버전은 `move_robot_joint_delta` 로 매 emit 마다 ±delta 를 부호만
    뒤집어 쏘는 패턴이었는데, 그러면 실제 로봇은 amplitude 안에서 떨기만 해서
    영상 재생할 때 사실상 정지로 보인다. 지금은 `move_robot_joint` 로
    home_pose 주변을 큰 진폭의 사인파로 휘젓도록 절대 타겟을 보낸다 — 시간
    기반 위상이라 emit rate 와 관계없이 일정한 궤적을 그린다.
    """

    # joint 별 (위상 진폭, 진동수[Hz]) — joint1/2/3/5 만 움직이고 joint6/gripper 는
    # 기본값 유지. 진폭은 tutorial_arm joint bound 안에 들어오게 보수적으로 잡았다.
    _JOINT_OSC = [
        (0.6, 0.20),   # joint1 — yaw
        (0.4, 0.25),   # joint2
        (0.4, 0.30),   # joint3
        (0.0, 0.0),    # joint4 (정지)
        (0.5, 0.35),   # joint5
        (0.0, 0.0),    # joint6
        (0.0, 0.0),    # gripper (tool)
    ]

    def __init__(self, backend_url, robot_id, joint_dim, base_pose, hz=10):
        self.backend_url = backend_url.rstrip('/')
        self.robot_id = robot_id
        self.joint_dim = joint_dim
        # base_pose 가 누락되거나 길이가 다르면 0 으로 패딩
        base = list(base_pose or [])
        if len(base) < joint_dim:
            base = base + [0.0] * (joint_dim - len(base))
        self.base_pose = base[:joint_dim]
        self.hz = hz
        self._stop = threading.Event()
        self._thread = None
        self._sio = socketio.Client(reconnection=True, logger=False, engineio_logger=False)
        self._t0 = None

    def start(self):
        self._sio.connect(self.backend_url, wait=True, transports=['websocket', 'polling'])
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._run, name='KeyboardInjector', daemon=True)
        self._thread.start()

    def _target(self, elapsed):
        import math
        target = list(self.base_pose)
        for i in range(min(self.joint_dim, len(self._JOINT_OSC))):
            amp, freq = self._JOINT_OSC[i]
            if amp == 0.0 or freq == 0.0:
                continue
            target[i] = self.base_pose[i] + amp * math.sin(2 * math.pi * freq * elapsed)
        return target

    def _run(self):
        period = 1.0 / max(1, self.hz * 2)  # record 루프보다 2배 빠르게
        while not self._stop.is_set():
            elapsed = time.time() - (self._t0 or time.time())
            target = self._target(elapsed)
            try:
                self._sio.emit('move_robot_joint', {
                    'robot': {'id': self.robot_id},
                    'goal_pos': target,
                })
            except Exception as e:  # noqa: BLE001
                # socketio가 일시적으로 끊겨도 다음 사이클에 재연결 시도
                print(f'  [KeyboardInjector] emit failed: {e}')
            time.sleep(period)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self._sio.disconnect()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Recording flow
# ---------------------------------------------------------------------------

def record_episodes(api: Backend, backend_url, workspace, dataset_id,
                     robot, episodes, hz, episode_seconds):
    """tele_type='keyboard' 로 episodes 만큼 에피소드를 녹화한다.

    각 에피소드는 workspace.episode_len 만큼 (기본 200 step) 기록되며,
    record_episode 의 max_timesteps for-loop 이 자연 종료되면 episode_saved
    이벤트가 발생한다. 강제 complete_episode 는 호출하지 않는다 — 그렇게
    하면 inner while 의 첫 break 에서 outer for 가 즉시 끝나 length=1 만
    저장되기 때문 (record_episode.py:300, 312 참고).

    `episode_seconds` 는 이전 호환을 위해 받지만, 자연 종료 timeout 의
    하한으로만 쓰인다 (실제 대기 시간 = max(episode_len/hz + 60s,
    episode_seconds + 60s)).
    """
    sensors = workspace['sensors']
    robots = workspace['assembly']['robots']

    saved = []

    saved_event = threading.Event()
    sio = socketio.Client(reconnection=True, logger=False, engineio_logger=False)

    @sio.on('episode_saved')
    def _on_saved(data):
        print(f'  episode_saved: {data}')
        saved_event.set()

    @sio.on('moving_homepose')
    def _on_homepose(data):
        if data.get('moving'):
            print('  moving to homepose ...')

    sio.connect(backend_url, wait=True, transports=['websocket', 'polling'])

    try:
        joint_dim = len(robot.get('joint_names') or []) or 7
        # 사인파의 중심을 home_pose 로 두면 record_episode 가 매 에피소드 직전에
        # move_homepose 로 복귀시킨 직후의 자세와 자연스럽게 이어진다.
        home_pose_map = workspace.get('home_pose') or {}
        base_pose = home_pose_map.get(str(robot['id'])) or [0.0] * joint_dim
        injector = KeyboardInjector(backend_url, robot['id'], joint_dim,
                                     base_pose=base_pose, hz=hz)
        injector.start()
        try:
            episode_len = int(workspace.get('episode_len') or 200)
            # 자연 종료까지 걸리는 시간: keyboard tele_type 은 키 입력 1개당
            # 1 step 이라 hz 와 무관하게 KeyboardInjector 가 키를 보내는
            # 페이스에 종속된다. 안전하게 episode_len/hz + 60s, 그리고
            # 옛 episode_seconds 인자 + 60s 중 큰 쪽으로.
            natural_secs = episode_len / max(hz, 1)
            wait_timeout = max(natural_secs, episode_seconds) + 60.0
            for ep_idx in range(episodes):
                saved_event.clear()
                payload = {
                    'task': workspace,
                    'robots': robots,
                    'sensors': sensors,
                    'tele_type': 'keyboard',
                    'assembly_id': workspace['assembly_id'],
                    'move_homepose': True,
                    'hz': hz,
                    'language_instruction': 'tutorial keyboard demo',
                    # 한 번 호출에 한 에피소드만 저장.
                    'iter': 1,
                }
                print(f'  [ep {ep_idx + 1}/{episodes}] start_collection '
                      f'(target {episode_len} steps @ {hz}Hz, timeout {wait_timeout:.0f}s)')
                api.post(f'/dataset/{dataset_id}/:start_collection', json=payload)

                # complete_episode 를 강제로 보내지 않는다 — record_episode 의
                # for t in range(max_timesteps) 가 episode_len 만큼 자연
                # 종료하도록 둔다.
                if not saved_event.wait(timeout=wait_timeout):
                    raise StageError(
                        f'episode_saved 이벤트가 {wait_timeout:.0f}초 내에 오지 않음 (ep {ep_idx + 1})')
                saved.append(ep_idx)

                # collection 루프가 자연 종료할 시간 주기
                time.sleep(1.0)
                api.post(f'/dataset/{dataset_id}/:stop_collection')
                time.sleep(1.0)
        finally:
            injector.stop()
    finally:
        try:
            sio.disconnect()
        except Exception:
            pass

    if len(saved) != episodes:
        raise StageError(f'expected {episodes} episodes, only saved {len(saved)}')


# ---------------------------------------------------------------------------
# Training flow
# ---------------------------------------------------------------------------

def create_policy(api: Backend, name='tutorial_act_policy', action_type='qaction',
                  obs_state_keys=None):
    """ACT 정책 — TrainPage 가 기본으로 만드는 형태와 동일한 settings.

    action_type 으로 학습/추론 시 action 표현을 바꾼다. 그 외 settings 는 모든
    action_type 에서 동일 (smoke test 용으로 dim_model/dim_feedforward 를
    프론트 기본값보다 작게 잡아 학습을 빠르게 한다):
      - 'qaction'        : 절대 joint position target (action.joint 컬럼).
      - 'relative_ee_pos': EE local-frame chunk trajectory. 학습/추론 모두
                            observation.eepos 시간 차분으로 derive 하므로
                            dataset 에 observation.eepos 가 있어야 한다
                            (tutorial_arm 은 ik_setting 보유 → record 시 저장됨).

    obs_state_keys 는 None 이면 프론트 ACT 기본값 ['qpos'] 를 쓴다. relative_ee_pos
    의 anchor(observation.eepos)는 obs_state_keys 와 독립적인 별도 채널이라
    obs_state_keys 를 바꿀 필요는 없다.
    """
    settings = {
        'n_obs_steps': 1,
        'chunk_size': 15,
        'n_action_steps': 1,
        'temporal_ensemble_coeff': 0.01,
        'vision_backbone': 'resnet18',
        'pretrained_backbone_weights': 'ResNet18_Weights.IMAGENET1K_V1',
        'replace_final_stride_with_dilation': False,
        'pre_norm': False,
        'dim_model': 256,
        'n_heads': 8,
        'dim_feedforward': 1024,
        'feedforward_activation': 'relu',
        'n_encoder_layers': 4,
        'n_decoder_layers': 1,
        'use_vae': True,
        'latent_dim': 32,
        'n_vae_encoder_layers': 4,
        'action_type': action_type,
        'obs_state_keys': list(obs_state_keys or ['qpos']),
    }
    res = api.post('/policy', json={
        'name': name,
        'type': 'ACT',
        'settings': settings,
    })
    if res.get('status') != 'success':
        raise StageError(f'create policy failed: {res}')
    return res['data']


def create_checkpoint_and_train(api: Backend, workspace_id, policy_id,
                                  dataset_id, episodes, training_server_url,
                                  num_epochs=100, batch_size=4):
    train_settings = {
        'num_epochs': num_epochs,
        'batch_size': batch_size,
        'num_workers': 2,
        'use_peft': False,
        'use_amp': False,
        'grad_accum_steps': 1,
        'dropout': 0.1,
        'kl_weight': 10,
        'optimizer_lr': 1e-5,
        'optimizer_weight_decay': 1e-4,
        'optimizer_lr_backbone': 1e-5,
    }
    dataset_info = {
        str(dataset_id): {'episode_num': episodes},
    }
    res = api.post('/checkpoint', json={
        'name': f'tutorial_e2e_{int(time.time())}',
        'task_id': workspace_id,
        'policy_id': policy_id,
        'dataset_info': dataset_info,
        'train_settings': train_settings,
    })
    if res.get('status') != 'success':
        raise StageError(f'create checkpoint failed: {res}')
    checkpoint_id = res['id']
    print(f'  checkpoint_id={checkpoint_id}')

    # 학습 서버 health 먼저 체크 (학습 enqueue 전에 빠르게 실패하기 위해)
    health = api.post('/remote-train/health', json={'server_url': training_server_url})
    if health.get('status') != 'success':
        raise StageError(f'training_server health 실패: {health}')

    enq = api.post('/train/queue', json={
        'server_url': training_server_url,
        'checkpoint_id': checkpoint_id,
    })
    if enq.get('status') != 'success':
        raise StageError(f'train enqueue failed: {enq}')
    print(f'  enqueued: {enq}')
    return checkpoint_id


def wait_for_training(api: Backend, checkpoint_id, max_wait_seconds=3600):
    """status가 finished 가 될 때까지 polling. failed/canceled 면 즉시 실패."""
    deadline = time.time() + max_wait_seconds
    last_status = None
    while time.time() < deadline:
        snap = api.get('/train/queue')
        running = snap.get('running')
        recent = snap.get('recent', [])
        for entry in [running] + list(recent):
            if not entry:
                continue
            if entry.get('id') == checkpoint_id:
                status = entry.get('status')
                if status != last_status:
                    print(f'  checkpoint {checkpoint_id} status={status} '
                          f'progress={entry.get("progress")}')
                    last_status = status
                if status == 'finished':
                    return
                if status in ('failed', 'canceled'):
                    raise StageError(f'training ended in {status}: {entry}')
                break
        else:
            # snapshot에 없으면 직접 행을 조회
            ckpt = api.get(f'/checkpoint/{checkpoint_id}')
            ck = ckpt.get('checkpoint') if ckpt.get('status') == 'success' else None
            if ck and ck.get('status') == 'finished':
                return
            if ck and ck.get('status') in ('failed', 'canceled'):
                raise StageError(f'training ended in {ck.get("status")}: {ck}')
        time.sleep(5.0)
    raise StageError(f'training did not finish within {max_wait_seconds}s')


# ---------------------------------------------------------------------------
# Inference flow
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Planner flow (joint_position + timesleep + checkpoint)
# ---------------------------------------------------------------------------

def _make_block(btype, **fields):
    """plan 블록은 frontend 가 만드는 모양과 호환되도록 id 를 같이 박는다.
    backend(planner.py / planner_run.py)는 id 가 없어도 동작하지만, end-event
    로그가 더 알아보기 쉬워지므로 채워둔다."""
    block = {'id': f'{btype}_{int(time.time() * 1000)}_{id(fields) & 0xfff:x}', 'type': btype}
    block.update(fields)
    return block


def run_planner(api: Backend, backend_url, workspace, robot, checkpoint_id,
                checkpoint_name, ckpt_duration=8.0, sleep_seconds=2.0,
                wait_timeout=180.0):
    """3 block plan 을 만들어 :start_run 후 planner_run_end='finished' 대기.

    block 1) joint_position — workspace의 home_pose 로 단발 이동
    block 2) timesleep      — sleep_seconds 만큼 대기
    block 3) checkpoint     — 학습된 ckpt 를 ckpt_duration 동안 추론
    """
    home_pose_map = workspace.get('home_pose') or {}
    if str(robot['id']) not in home_pose_map:
        raise StageError(f'workspace home_pose missing entry for robot {robot["id"]}: '
                         f'{home_pose_map}')

    plan = [
        _make_block(
            'joint_position',
            workspace_id=workspace['id'],
            positions={str(robot['id']): list(home_pose_map[str(robot['id'])])},
            name='go_home',
        ),
        _make_block(
            'timesleep',
            duration=sleep_seconds,
            name='settle',
        ),
        _make_block(
            'checkpoint',
            workspace_id=workspace['id'],
            checkpoint_id=checkpoint_id,
            checkpoint_name=checkpoint_name,
            duration=ckpt_duration,
            hz=10,
            re_inference_steps=1,
            temporal_ensemble_coeff=0.01,
            name=f'run {checkpoint_name}',
        ),
    ]

    create_res = api.post('/planner', json={
        'name': f'tutorial_e2e_planner_{int(time.time())}',
        'task_ids': [workspace['id']],
    })
    if create_res.get('status') != 'success':
        raise StageError(f'create planner failed: {create_res}')
    planner_id = create_res['id']

    # 백엔드 planner 는 flat `plan` 대신 그룹 구조(`plans`)로 블록을 저장한다.
    # POST /planner 는 `plans` 만 읽어 `plan` 키는 무시되므로, 레거시 호환
    # 경로인 PUT {'plan': ...} 로 flat plan 을 보내 _rebalance_plans 가
    # 각 블록을 workspace_id 기준 그룹에 분배하게 한다 (planner.py:317-321).
    upd_res = api.put(f'/planner/{planner_id}', json={'plan': plan})
    if upd_res.get('status') != 'success':
        raise StageError(f'planner plan update failed: {upd_res}')
    print(f'  planner_id={planner_id} (3 blocks: joint_position → timesleep → checkpoint)')

    # planner_run_end 이벤트로 종료를 감지 — 폴링 + run_status 도 보조
    end_event = threading.Event()
    end_payload = {}

    sio = socketio.Client(reconnection=True, logger=False, engineio_logger=False)

    @sio.on('planner_block_start')
    def _on_block_start(data):
        print(f'  [block {data.get("index", "?") + 1 if isinstance(data.get("index"), int) else "?"}/'
              f'{data.get("total")}] start: {data.get("type")} — {data.get("name")}')

    @sio.on('planner_block_end')
    def _on_block_end(data):
        print(f'  [block {data.get("index", "?") + 1 if isinstance(data.get("index"), int) else "?"}/'
              f'{data.get("total")}] end:   status={data.get("status")} '
              f'error={data.get("error")}')

    @sio.on('planner_run_end')
    def _on_run_end(data):
        end_payload.update(data or {})
        print(f'  planner_run_end: {data}')
        end_event.set()

    sio.connect(backend_url, wait=True, transports=['websocket', 'polling'])

    try:
        # 빈 body 라도 json={} 를 명시해야 Flask 가 415 안 냄.
        start_res = api.post(f'/planner/{planner_id}/:start_run', json={})
        if start_res.get('status') != 'success':
            raise StageError(f'start_run failed: {start_res}')

        # planner_run_end 가 안 오면(이벤트 손실 등) :run_status 폴링으로 보조
        deadline = time.time() + wait_timeout
        while not end_event.is_set() and time.time() < deadline:
            if end_event.wait(timeout=2.0):
                break
            status_res = api.get(f'/planner/{planner_id}/:run_status')
            if status_res.get('is_running') is False:
                # 백엔드는 끝났는데 socketio 이벤트가 누락된 경우. 짧게 한 번 더 기다린 뒤 종료.
                print('  [WARN] planner already not running on backend but no end event; waiting briefly')
                end_event.wait(timeout=2.0)
                break

        if not end_event.is_set():
            # 타임아웃 — 정리 차원에서 stop 호출
            try:
                api.post(f'/planner/{planner_id}/:stop_run', json={})
            except Exception:
                pass
            raise StageError(f'planner did not finish within {wait_timeout}s '
                             f'(planner_id={planner_id})')

        if end_payload.get('status') != 'finished':
            raise StageError(f'planner ended with status={end_payload.get("status")} '
                             f'error={end_payload.get("error")}')
    finally:
        try:
            sio.disconnect()
        except Exception:
            pass


def run_inference(api: Backend, checkpoint_id, workspace, robot, run_seconds=15):
    sensors = workspace['sensors']
    robots = workspace['assembly']['robots']
    ck = api.get(f'/checkpoint/{checkpoint_id}')
    if ck.get('status') != 'success':
        raise StageError(f'checkpoint {checkpoint_id} fetch failed: {ck}')
    policy = ck['checkpoint']['policy']
    if not policy:
        raise StageError('checkpoint has no policy attached')

    payload = {
        'task': workspace,
        'policy': policy,
        'robot_ids': [robot['id']],
        'sensors': sensors,
        'timesteps': 200,
        'move_homepose': False,
        'hz': 10,
        're_inference_steps': 1,
        'temporal_ensemble_coeff': 0.01,
        'inference_episode_len': 100,
    }
    # robots 키도 일부 코드 경로에서 사용될 수 있으나 start_test 라우트는
    # robot_ids 만 본다.
    payload['robots'] = robots

    res = api.post(f'/checkpoint/{checkpoint_id}/:start_test', json=payload)
    if res.get('status') != 'success':
        raise StageError(f'start_test failed: {res}')
    try:
        time.sleep(run_seconds)
    finally:
        stop_res = api.post(f'/checkpoint/{checkpoint_id}/:stop_test')
        if stop_res.get('status') != 'success':
            print(f'  [WARN] stop_test returned: {stop_res}')


# ---------------------------------------------------------------------------
# Per-action-type train → inference → planner phase
# ---------------------------------------------------------------------------

# 프론트 ACT action_type select 옵션과 1:1 — 테스트가 지원하는 값.
SUPPORTED_ACTION_TYPES = ('qaction', 'relative_ee_pos')


def run_action_type_phase(api: Backend, args, workspace, workspace_id, robot,
                          dataset_id, action_type):
    """하나의 action_type 에 대해 policy 생성 → 학습 → 추론 → planner 까지 실행.

    녹화 단계(record_episodes)는 action_type 와 무관하게 한 번만 돌고, 그
    데이터셋을 모든 action_type 학습이 공유한다. checkpoint/추론/planner 는
    policy 행에 박힌 action_type 만 보고 분기하므로 이 함수는 action_type 별로
    독립적인 policy/checkpoint 만 새로 만들면 된다.

    반환: 생성된 checkpoint_id (planner 까지 끝난 뒤).
    """
    label = action_type

    with stage(f'[{label}] create ACT policy'):
        policy = create_policy(
            api,
            name=f'tutorial_act_{label}_{int(time.time())}',
            action_type=action_type,
        )
        print(f'  policy_id={policy["id"]} action_type={action_type}')

    with stage(f'[{label}] create checkpoint + enqueue training '
               f'(epochs={args.train_epochs})'):
        checkpoint_id = create_checkpoint_and_train(
            api, workspace_id, policy['id'], dataset_id, args.episodes,
            args.training_server, num_epochs=args.train_epochs,
            batch_size=args.batch_size,
        )

    with stage(f'[{label}] wait for training to finish (checkpoint={checkpoint_id})'):
        wait_for_training(api, checkpoint_id)

    if not args.skip_inference:
        with stage(f'[{label}] run inference for {args.inference_seconds}s'):
            run_inference(api, checkpoint_id, workspace, robot,
                          run_seconds=args.inference_seconds)
    else:
        print(f'\n[INFO] [{label}] --skip-inference 으로 추론 단계 생략')

    if args.skip_planner:
        print(f'\n[OK] [{label}] inference finished; --skip-planner 이므로 planner 생략')
        return checkpoint_id

    # planner 의 checkpoint 블록은 내부적으로 /checkpoint/<id>/:start_test 와
    # 동일한 checkpoint_test 함수를 호출한다. 추론 단계가 직전에 끝났더라도
    # planner 호출 전에 잠깐 텀을 둬 GPU/메모리 정리 시간을 준다.
    time.sleep(2.0)

    # planner 의 checkpoint name 은 checkpoint dict에서 채워온다.
    ck = api.get(f'/checkpoint/{checkpoint_id}')
    checkpoint_name = (ck.get('checkpoint') or {}).get('name') or f'ckpt_{checkpoint_id}'

    with stage(f'[{label}] run planner (joint_position → timesleep '
               f'{args.planner_sleep_seconds}s → checkpoint '
               f'{args.planner_checkpoint_seconds}s)'):
        run_planner(
            api, args.backend, workspace, robot, checkpoint_id, checkpoint_name,
            ckpt_duration=args.planner_checkpoint_seconds,
            sleep_seconds=args.planner_sleep_seconds,
        )

    return checkpoint_id


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _teardown_robot(api: Backend, robot_id):
    """/robot:stop 호출. backend 가 응답 못해도 치명적이지 않음 (backend 가 죽으면
    robot 프로세스도 같이 죽음).
    """
    if not robot_id:
        return
    print('\n=== [TEARDOWN] stop robot driver ...', flush=True)
    try:
        res = api.post('/robot:stop', json={
            'id': robot_id, 'process_id': f'robot_{robot_id}',
        }, timeout=15)
        print(f'  /robot:stop -> {res}')
    except Exception as e:  # noqa: BLE001
        print(f'  [WARN] /robot:stop raised: {e}')
    print('=== [TEARDOWN robot done]', flush=True)


def _teardown_headless_sim(api: Backend):
    """--headless-sim 으로 띄운 mujoco sim을 stop. 어떤 단계 실패에도 finally
    로부터 호출되므로 backend health 자체가 실패해서 sim이 안 떴을 수도 있다.
    그래도 idempotent — TUTORIAL_LAUNCH_PROCESS_ID 가 없으면 bridge driver가
    조용히 무시한다.
    """
    print('\n=== [TEARDOWN] stop headless mujoco sim ...', flush=True)
    try:
        res = api.post('/tutorial:stop', timeout=15)
        print(f'  /tutorial:stop -> {res}')
    except StageError as e:
        # 백엔드 자체가 죽었거나 응답 실패 — sim 프로세스는 어차피 backend
        # 와 같이 죽으므로 치명적이지 않음.
        print(f'  [WARN] stop failed: {e}')
    except Exception as e:  # noqa: BLE001
        print(f'  [WARN] stop raised: {e}')
    print('=== [TEARDOWN done]', flush=True)


def _run_pipeline(api: Backend, args):
    """실제 stage 실행. teardown 분리를 위해 main 에서 분리."""
    with stage('backend health'):
        # 가장 가벼운 GET — robots 목록은 항상 떠 있으면 200을 돌려준다.
        api.get('/robots')

    tutorial_info = None
    with stage(f'tutorial seed + sim start{" (headless)" if args.headless_sim else ""}'):
        tutorial_info = ensure_tutorial_running(
            api,
            show_viewer=False if args.headless_sim else None,
        )

    robot_id = tutorial_info['robot_id']
    workspace_id = tutorial_info['workspace_id']

    with stage('cleanup test_by_code_* workspaces + clone tutorial_env'):
        deleted = cleanup_test_workspaces(api)
        if deleted:
            print(f'  deleted {len(deleted)} stale test_by_code_* workspace(s): {deleted}')
        else:
            print('  no stale test_by_code_* workspaces')
        new_name = f'test_by_code_{time.strftime("%Y%m%d_%H%M%S")}'
        workspace_id = clone_workspace(api, workspace_id, new_name)
        print(f'  cloned tutorial_env → {new_name} (id={workspace_id})')

    with stage(f'subscribe tutorial robot id={robot_id}'):
        subscribe_robot(api, robot_id)
        time.sleep(2.0)  # subscribe 워커가 첫 joint_state 받을 때까지 잠깐 대기

    workspace = None
    with stage(f'fetch workspace id={workspace_id}'):
        workspace = fetch_workspace_payload(api, workspace_id)
        # robots 리스트가 단일 single-arm 인지 sanity check
        if len(workspace['assembly']['robots']) != 1:
            raise StageError(f'expected single robot in tutorial assembly, '
                             f'got {len(workspace["assembly"]["robots"])}')
        if len(workspace['sensors']) < 2:
            raise StageError(f'expected >=2 tutorial sensors, '
                             f'got {len(workspace["sensors"])}')

    robot = workspace['assembly']['robots'][0]

    with stage(f'/robot:start id={robot["id"]}'):
        start_robot_driver(api, robot)
        time.sleep(1.0)

    dataset_id = None
    with stage('create dataset'):
        ds = api.post('/dataset', json={
            'name': f'tutorial_e2e_{int(time.time())}',
            'task_id': workspace_id,
        })
        if ds.get('status') != 'success':
            raise StageError(f'create dataset failed: {ds}')
        dataset_id = ds['dataset_id']
        print(f'  dataset_id={dataset_id}')

    with stage(f'record {args.episodes} episodes via keyboard mock'):
        record_episodes(api, args.backend, workspace, dataset_id, robot,
                         args.episodes, args.hz, args.episode_seconds)

    if args.skip_train:
        print('\n[OK] data collection succeeded; --skip-train 이므로 종료')
        return

    # action_type 별로 policy → 학습 → 추론 → planner 를 반복한다. 녹화된
    # 데이터셋 하나(observation.eepos 포함)를 모든 action_type 가 공유한다.
    print(f'\n[INFO] action_types to run: {args.action_types}')
    for action_type in args.action_types:
        run_action_type_phase(api, args, workspace, workspace_id, robot,
                              dataset_id, action_type)

    print('\n[OK] Easy Trainer end-to-end pipeline '
          f'(action_types={args.action_types}, incl. planner) completed without errors.')


def main():
    parser = argparse.ArgumentParser(description='Easy Trainer end-to-end smoke test')
    parser.add_argument('--backend', default=DEFAULT_BACKEND)
    parser.add_argument('--training-server', default=DEFAULT_TRAINING_SERVER)
    parser.add_argument('--episodes', type=int, default=2)
    parser.add_argument('--episode-seconds', type=float, default=4.0,
                        help='한 에피소드 동안 키보드 모방 입력을 보낼 시간')
    parser.add_argument('--hz', type=int, default=10, help='record/inference Hz')
    parser.add_argument('--train-epochs', type=int, default=100)
    parser.add_argument('--batch-size', type=int, default=4)
    parser.add_argument('--action-types', nargs='+', default=list(SUPPORTED_ACTION_TYPES),
                        choices=list(SUPPORTED_ACTION_TYPES),
                        help='학습/추론/planner 를 반복할 action_type 목록 '
                             '(기본: qaction relative_ee_pos)')
    parser.add_argument('--inference-seconds', type=float, default=15.0)
    parser.add_argument('--planner-checkpoint-seconds', type=float, default=8.0,
                        help='planner의 checkpoint 블록 duration')
    parser.add_argument('--planner-sleep-seconds', type=float, default=2.0,
                        help='planner의 timesleep 블록 duration')
    parser.add_argument('--skip-train', action='store_true',
                        help='학습/추론/planner 단계를 건너뛰고 데이터 수집까지만 검증')
    parser.add_argument('--skip-inference', action='store_true')
    parser.add_argument('--skip-planner', action='store_true',
                        help='planner 단계를 건너뛴다')
    parser.add_argument('--headless-sim', action='store_true',
                        help='mujoco viewer 창 없이 sim 기동 (headless). 종료 시 sim도 stop.')
    parser.add_argument('--keep-sim', action='store_true',
                        help='--headless-sim 이어도 종료 시 sim 을 stop 하지 않음 (디버깅용)')
    args = parser.parse_args()

    api = Backend(args.backend)
    started_robot_id = None
    try:
        # tutorial_status 로 robot_id 를 미리 확보 — 실패해도 무시. 종료 시
        # _teardown_robot 으로 robot:start 가 띄운 interp_node 까지 정리하기 위함.
        try:
            st = api.get('/tutorial/status', timeout=5)
            started_robot_id = st.get('robot_id')
        except Exception:
            pass
        _run_pipeline(api, args)
    finally:
        # robot driver(+interp_node) 정리. /robot:start 가 호출되지 않았어도
        # idempotent (없는 process 는 bridge 가 조용히 무시).
        _teardown_robot(api, started_robot_id)
        # --headless-sim 으로 띄운 sim 은 자동화 종료 후 정리한다.
        # 명시적으로 viewer 모드로 띄웠거나 사용자가 띄워둔 채 디버깅 중일 수
        # 있는 경우(=> --headless-sim 미지정)에는 건드리지 않는다.
        if args.headless_sim and not args.keep_sim:
            _teardown_headless_sim(api)


if __name__ == '__main__':
    main()
