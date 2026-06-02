# -*- coding: utf-8 -*-
"""
DriverService gRPC servicer.
ROS2 컨테이너에서 ros2 launch/run 프로세스를 관리한다.
기존 robot.py / sensor.py의 드라이버 시작 로직을 추출.
"""
import json
import os
import re
import signal
import subprocess
import threading
import time

import grpc

from ..generated import robot_bridge_pb2 as pb
from ..generated import robot_bridge_pb2_grpc as pb_grpc
from ..configs.module_loader import (
    get_robot_driver_launch,
    get_sensor_driver_launch,
    get_robot_driver_hooks,
    get_robot_driver_remote,
)


# {key} 또는 {key|default}
_LAUNCH_PLACEHOLDER_RE = re.compile(r"\{([a-zA-Z_][a-zA-Z0-9_]*)(?:\|([^}]*))?\}")


def _substitute_launch_args(value: str, ctx: dict) -> str:
    """문자열의 `{key}` / `{key|default}` 플레이스홀더를 ctx 에서 치환.
    ctx 에 키가 없거나 빈 값이고 default 가 있으면 default 사용. 둘 다 없으면 원본 유지."""
    def repl(m):
        k = m.group(1)
        default = m.group(2)
        v = ctx.get(k)
        if v is not None and v != "":
            return str(v)
        if default is not None:
            return default
        return m.group(0)
    return _LAUNCH_PLACEHOLDER_RE.sub(repl, value)


def _build_ctx(entity_kind: str, entity_id: int, settings: dict) -> dict:
    """launch args 치환용 컨텍스트. 표준 placeholder + 정규화."""
    ctx = dict(settings or {})
    # CAN 인터페이스 이름 정규화 (can_X → canX)
    cp = ctx.get('can_port')
    if isinstance(cp, str) and cp.startswith('can_'):
        ctx['can_port'] = 'can' + cp[4:]
    if entity_kind == 'sensor':
        ctx['sensor_id'] = entity_id
        ctx['namespace'] = f'ec_sensor_{entity_id}'
    else:
        ctx['robot_id'] = entity_id
        ctx['namespace'] = f'ec_robot_{entity_id}'
    return ctx


def _build_argv_from_launch(launch: dict, ctx: dict) -> list[str] | None:
    """module.json `driver.launch` 블록에서 ros2 launch / ros2 run argv 를 만든다.

    스키마:
      {"command": "launch"|"run", "package": str,
       "launch_file": str (launch 일 때),
       "executable": str (run 일 때),
       "args": {key: value_template, ...}}

    `command` 필드 생략 시 기본 'launch'.
    """
    if not isinstance(launch, dict):
        return None
    pkg = launch.get('package', '').strip()
    if not pkg:
        return None
    cmd = (launch.get('command') or 'launch').strip().lower()
    args = launch.get('args') or {}

    if cmd == 'launch':
        lfile = launch.get('launch_file', '').strip()
        if not lfile:
            return None
        argv = ['ros2', 'launch', pkg, lfile]
        for k, v in args.items():
            if not k:
                continue
            argv.append(f'{k}:={_substitute_launch_args(str(v), ctx)}')
        return argv
    if cmd == 'run':
        executable = launch.get('executable', '').strip()
        if not executable:
            return None
        argv = ['ros2', 'run', pkg, executable]
        # run 은 ros_args 별도 처리: --ros-args 뒤에 -p / -r 등 붙임
        ros_args = launch.get('ros_args') or {}
        if ros_args:
            argv.append('--ros-args')
            for k, v in ros_args.items():
                if not k:
                    continue
                argv += ['-p', f'{k}:={_substitute_launch_args(str(v), ctx)}']
        return argv
    return None


# ---------------------------------------------------------------------------
# Remote (SSH) robot driver helpers
#
# 온보드 PC 를 가진 로봇(예: ROBOTIS OMY)은 드라이버를 로봇 PC 에서 직접 실행하고
# EasyTrainer 는 같은 DDS 도메인에서 토픽만 주고받는다. driver.remote 블록이
# 있고 settings 의 host_field(기본 ssh_host)가 채워져 있으면 아래 경로로 동작한다.
#   1) payload 파일 scp (있으면)
#   2) provision step 순차 실행 (check 성공 시 skip → 멱등)
#   3) launch 명령을 ssh -tt 로 blocking 실행 (tracked subprocess)
# 토픽은 DDS 로 흐르므로 보간 노드/agent 는 로컬 모드와 동일하게 동작한다.
# ---------------------------------------------------------------------------

def _remote_is_active(remote: dict | None, settings: dict) -> bool:
    if not isinstance(remote, dict):
        return False
    host_field = remote.get('host_field', 'ssh_host')
    return bool((settings or {}).get(host_field))


def _remote_conn(remote: dict, settings: dict) -> tuple[str, str, str]:
    """(user, host, port) 추출."""
    host = settings.get(remote.get('host_field', 'ssh_host')) or ''
    user = settings.get(remote.get('user_field', 'ssh_user')) or remote.get('default_user', 'root')
    port = settings.get(remote.get('port_field', 'ssh_port')) or remote.get('default_port', 22)
    return str(user), str(host), str(port)


def _sshpass_prefix(remote: dict, settings: dict) -> tuple[list[str], bool]:
    """password_field(기본 ssh_password) 가 채워져 있으면 (['sshpass','-p',pw], True),
    아니면 ([], False). 암호 인증은 sshpass 가 ros2 컨테이너에 설치돼 있어야 한다
    (모듈 apt deps 에 'sshpass' 추가)."""
    pw = (settings or {}).get(remote.get('password_field', 'ssh_password'))
    if pw:
        return ['sshpass', '-p', str(pw)], True
    return [], False


def _ssh_auth_opts(use_pw: bool) -> list[str]:
    """인증 방식별 ssh/scp 공통 옵션."""
    if use_pw:
        # sshpass 가 비밀번호 프롬프트를 먹이도록 password 인증 강제. 키 우선 시도로
        # 프롬프트가 안 떠 sshpass 가 멈추는 것을 막는다.
        return ['-o', 'PreferredAuthentications=password,keyboard-interactive',
                '-o', 'PubkeyAuthentication=no']
    # 키 기반: 비밀번호 프롬프트로 멈추지 않도록 BatchMode.
    return ['-o', 'BatchMode=yes']


def _ssh_base(remote: dict, settings: dict, interactive_tty: bool) -> list[str]:
    user, host, port = _remote_conn(remote, settings)
    prefix, use_pw = _sshpass_prefix(remote, settings)
    argv = list(prefix) + ['ssh']
    # -tt: launch(장기 실행) 는 TTY 강제 → 로컬 ssh 종료 시 원격 프로세스 트리에
    # SIGHUP 전파. provision(단발) 은 -T 로 충분.
    argv.append('-tt' if interactive_tty else '-T')
    argv += [
        '-o', 'StrictHostKeyChecking=accept-new',
        '-o', 'ServerAliveInterval=10',
        '-o', 'ServerAliveCountMax=3',
        '-p', port,
    ]
    argv += _ssh_auth_opts(use_pw)
    argv += [f'{user}@{host}']
    return argv


def _remote_inner(remote: dict, body: str, ctx: dict) -> str:
    """원격에서 실행할 한 줄 sh. ros_domain_id export 를 앞에 붙이고 placeholder 치환."""
    dom = remote.get('ros_domain_id')
    prefix = f'export ROS_DOMAIN_ID={dom}; ' if dom is not None else ''
    return _substitute_launch_args(prefix + body, ctx)


class DriverServiceServicer(pb_grpc.DriverServiceServicer):
    def __init__(self, node):
        """
        Args:
            node: rclpy.node.Node - ROS2 node for topic listing.
        """
        self.node = node
        self.processes = {}  # name -> subprocess.Popen
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Robot Driver
    # ------------------------------------------------------------------
    def StartRobotDriver(self, request, context):
        settings = json.loads(request.settings_json) if request.settings_json else {}
        robot_id = request.robot_id
        company = request.company
        rtype = request.type
        process_id = request.process_id

        # 기존 프로세스 정리
        self._stop(process_id)

        # module.json 의 driver hooks (pre/post launch). 없으면 빈 dict.
        try:
            hooks = get_robot_driver_hooks(rtype) or {}
        except Exception as e:
            print(f"[driver_service] get_robot_driver_hooks 실패: {e}", flush=True)
            hooks = {}
        hook_module_id = hooks.get('module_id', '')

        ctx = _build_ctx('robot', robot_id, settings)

        # ── 원격(SSH) 드라이버 분기 ───────────────────────────────────────
        # driver.remote 가 있고 settings 에 ssh_host 가 차 있으면 로봇 PC 에서
        # 드라이버를 실행한다. 로컬 pre_launch / SDK 분기 / ros2 launch 는 건너뛴다.
        try:
            remote = get_robot_driver_remote(rtype)
        except Exception as e:
            print(f"[driver_service] get_robot_driver_remote 실패: {e}", flush=True)
            remote = None
        if _remote_is_active(remote, settings):
            return self._start_remote_robot_driver(
                remote, robot_id, rtype, process_id, settings, ctx)

        # Pre-launch hooks — driver 시작 전 (예: piper CAN setup script)
        for hook in hooks.get('pre_launch') or []:
            self._run_pre_launch_hook(hook, robot_id=robot_id, module_id=hook_module_id,
                                      ctx=ctx, log_name=process_id)

        # SDK 제어 로봇: ROS2 드라이버/보간 노드 불필요 (Agent가 SDK로 직접 제어)
        if settings.get('sdk_control'):
            proc = None
            # 프로세스 목록에 마커 등록 (status 확인용)
            with self._lock:
                self.processes[process_id] = None
        else:
            command = self._build_robot_command(company, rtype, robot_id, settings)
            if not command:
                return pb.DriverStatus(success=False, message=f'Unsupported robot type: {rtype}')
            proc = self._start_subprocess(process_id, command)
            if proc is None:
                return pb.DriverStatus(success=False, message='Failed to start process')

        # 보간 노드 시작: interpolation=True인 로봇
        if settings.get('interpolation'):
            self._spawn_interpolation_node_for_builtin(
                robot_id=robot_id, rtype=rtype, settings=settings,
                log_name=process_id,
            )

        # Post-launch hooks (예: JAKA servo enable ROS service call)
        for hook in hooks.get('post_launch') or []:
            self._run_post_launch_hook(hook, ctx=ctx)

        pid = proc.pid if proc else 0
        return pb.DriverStatus(success=True, message='Driver started', pid=pid)

    # ------------------------------------------------------------------
    # Remote (SSH) robot driver
    # ------------------------------------------------------------------
    def _start_remote_robot_driver(self, remote, robot_id, rtype, process_id, settings, ctx):
        """driver.remote 경로: 로봇 PC 에 SSH 로 provisioning 후 launch 실행."""
        user, host, port = _remote_conn(remote, settings)
        if not host:
            return pb.DriverStatus(success=False, message='ssh_host is empty')
        print(f"[remote] {rtype} via ssh {user}@{host}:{port}", flush=True)

        # 원격 모드 토픽 override (예: stock arm_controller 의 JointTrajectory).
        # 보간 노드/agent 가 이 settings 를 참조하므로 launch 전에 덮어쓴다.
        for key in ('read_topic', 'write_topic', 'write_topic_msg'):
            if remote.get(key):
                settings[key] = remote[key]

        # 1) payload 파일 scp (멱등 — 매번 덮어써도 무방)
        if not self._remote_copy_payload(remote, settings, log_name=process_id):
            return pb.DriverStatus(success=False, message='Remote payload copy failed')

        # 2) provision steps (check 성공 시 skip)
        ok, msg = self._remote_provision(remote, settings, ctx, log_name=process_id)
        if not ok:
            return pb.DriverStatus(success=False, message=f'Remote provisioning failed: {msg}')

        # 3) launch — blocking, tracked subprocess (ssh -tt)
        launch_body = (remote.get('launch') or '').strip()
        if not launch_body:
            return pb.DriverStatus(success=False, message='remote.launch is empty')
        inner = _remote_inner(remote, 'exec ' + launch_body, ctx)
        command = _ssh_base(remote, settings, interactive_tty=True) + [inner]
        proc = self._start_subprocess(process_id, command)
        if proc is None:
            return pb.DriverStatus(success=False, message='Failed to start remote launch')

        # 보간 노드 (override 된 토픽 사용) + post_launch hooks
        if settings.get('interpolation'):
            self._spawn_interpolation_node_for_builtin(
                robot_id=robot_id, rtype=rtype, settings=settings, log_name=process_id)
        try:
            hooks = get_robot_driver_hooks(rtype) or {}
        except Exception:
            hooks = {}
        for hook in hooks.get('post_launch') or []:
            self._run_post_launch_hook(hook, ctx=ctx)

        return pb.DriverStatus(success=True, message='Remote driver started', pid=proc.pid)

    def _remote_copy_payload(self, remote, settings, log_name) -> bool:
        """remote.payload 의 파일들을 scp 로 로봇 PC 에 복사. 없으면 no-op."""
        payload = remote.get('payload') or []
        if not payload:
            return True
        user, host, port = _remote_conn(remote, settings)
        for item in payload:
            src = (item.get('src') or '').strip()
            dst = (item.get('dst') or '').strip()
            if not src or not dst:
                continue
            if not os.path.exists(src):
                print(f"[remote/{log_name}] payload src not found: {src}", flush=True)
                return False
            # 원격 대상 디렉터리 보장
            rc = self._run_blocking(
                _ssh_base(remote, settings, interactive_tty=False)
                + [f'mkdir -p {self._shell_quote(os.path.dirname(dst))}'],
                log_name=log_name, timeout=30)
            if rc != 0:
                return False
            prefix, use_pw = _sshpass_prefix(remote, settings)
            scp = list(prefix) + ['scp', '-o', 'StrictHostKeyChecking=accept-new', '-P', port]
            scp += _ssh_auth_opts(use_pw)
            scp += ['-r', src, f'{user}@{host}:{dst}']
            rc = self._run_blocking(scp, log_name=log_name, timeout=300)
            if rc != 0:
                print(f"[remote/{log_name}] scp failed: {src} → {dst}", flush=True)
                return False
            print(f"[remote/{log_name}] payload copied: {os.path.basename(src)}", flush=True)
        return True

    def _remote_provision(self, remote, settings, ctx, log_name) -> tuple[bool, str]:
        """provision step 순차 실행. check 가 exit 0 이면 skip(멱등)."""
        for i, step in enumerate(remote.get('provision') or []):
            name = step.get('name') or f'step{i}'
            check = (step.get('check') or '').strip()
            run = (step.get('run') or '').strip()
            timeout = float(step.get('timeout', 600))
            optional = bool(step.get('optional'))
            if check:
                rc = self._run_blocking(
                    _ssh_base(remote, settings, interactive_tty=False)
                    + [_remote_inner(remote, check, ctx)],
                    log_name=log_name, timeout=min(timeout, 60))
                if rc == 0:
                    print(f"[remote/{log_name}] provision '{name}': already done — skip", flush=True)
                    continue
            if not run:
                continue
            print(f"[remote/{log_name}] provision '{name}': running…", flush=True)
            rc = self._run_blocking(
                _ssh_base(remote, settings, interactive_tty=True)
                + [_remote_inner(remote, run, ctx)],
                log_name=log_name, timeout=timeout)
            if rc != 0 and not optional:
                return False, f"step '{name}' exited {rc}"
        return True, ''

    def _run_blocking(self, command, log_name, timeout) -> int:
        """짧은 명령을 동기 실행하고 stdout/stderr 를 로그로 흘린다. 반환: returncode (timeout/예외 시 -1)."""
        command = ['stdbuf', '-oL'] + command if os.name != 'nt' else command
        try:
            proc = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=(os.name != 'nt'))
        except Exception as e:
            print(f"[remote/{log_name}] spawn failed: {e}", flush=True)
            return -1

        def _reader():
            try:
                for line in proc.stdout:
                    print(f"[{log_name}] {line.rstrip()}", flush=True)
            except Exception:
                pass
        t = threading.Thread(target=_reader, daemon=True)
        t.start()
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                proc.kill()
            print(f"[remote/{log_name}] command timed out after {timeout}s", flush=True)
            return -1
        t.join(timeout=2)
        return proc.returncode

    def StopRobotDriver(self, request, context):
        self._stop(request.name)
        # 보간 노드도 함께 정리 (robot_{id} → interp_{id})
        if request.name.startswith('robot_'):
            robot_id = request.name[len('robot_'):]
            self._stop(f'interp_{robot_id}')
        return pb.StatusResponse(success=True, message='Stopped')

    # ------------------------------------------------------------------
    # Sensor Driver
    # ------------------------------------------------------------------
    def StartSensorDriver(self, request, context):
        settings = json.loads(request.settings_json) if request.settings_json else {}
        sensor_id = request.sensor_id
        company = request.company
        stype = request.type
        process_id = request.process_id

        if stype == 'custom':
            return pb.DriverStatus(success=True, message='Custom sensor uses external topic')

        command = self._build_sensor_command(company, stype, sensor_id, settings)
        if not command:
            return pb.DriverStatus(success=False, message=f'Unsupported sensor company: {company}')

        proc = self._start_subprocess(process_id, command)
        if proc is None:
            return pb.DriverStatus(success=False, message='Failed to start sensor process')

        return pb.DriverStatus(success=True, message='Sensor driver started', pid=proc.pid)

    def StopSensorDriver(self, request, context):
        self._stop(request.name)
        return pb.StatusResponse(success=True, message='Stopped')

    # ------------------------------------------------------------------
    # Generic ros2 launch (used by tutorial mode / future sim modules)
    # ------------------------------------------------------------------
    def StartLaunch(self, request, context):
        process_id = request.process_id
        package = request.package
        launch_file = request.launch_file
        try:
            args = json.loads(request.args_json) if request.args_json else {}
        except json.JSONDecodeError as e:
            return pb.DriverStatus(success=False, message=f'Invalid args_json: {e}')

        if not package or not launch_file:
            return pb.DriverStatus(success=False, message='package and launch_file are required')

        # ros2 launch <pkg> <file> key:=val key:=val ...
        # 워크스페이스를 매번 source — bridge 기동 후에 colcon build 된 패키지도 인식
        launch_argv = ['ros2', 'launch', package, launch_file]
        for k, v in args.items():
            launch_argv.append(f'{k}:={v}')

        ws_setup = '/root/ros2_ws/install/setup.bash'
        bash_cmd = (
            'source /opt/ros/humble/setup.bash && '
            f'[ -f {ws_setup} ] && source {ws_setup}; '
            'exec ' + ' '.join(self._shell_quote(a) for a in launch_argv)
        )
        command = ['bash', '-lc', bash_cmd]

        # Replace any prior instance with the same process_id
        self._stop(process_id)
        proc = self._start_subprocess(process_id, command)
        if proc is None:
            return pb.DriverStatus(success=False, message='Failed to start launch')
        return pb.DriverStatus(success=True, message='Launch started', pid=proc.pid)

    def StopLaunch(self, request, context):
        self._stop(request.name)
        return pb.StatusResponse(success=True, message='Stopped')

    # ------------------------------------------------------------------
    # Interpolation Node (custom robots)
    # ------------------------------------------------------------------
    def _interp_process_id(self, robot_id):
        return f'interp_{robot_id}'

    def _build_interpolation_cmd(self, robot_id, extra_params):
        """interpolation_node 기동 argv 공통 빌더.

        ns = /ec_robot_<id>. agent.py 가 같은 ns 의 ec_joint_cmd 로 명령을 보내고
        보간 결과는 extra_params['output_topic'] 로 퍼블리시된다.
        """
        ns = f'ec_robot_{robot_id}'
        cmd = [
            'python3', '-m', 'ros2_bridge.interpolation_node',
            '--ros-args', '-r', f'__ns:=/{ns}',
        ]
        for k, v in extra_params.items():
            if v is None or v == '':
                continue
            cmd += ['-p', f'{k}:={v}']
        return cmd

    def _spawn_interpolation_node_for_builtin(self, robot_id, rtype, settings, log_name):
        """빌트인 로봇용 interpolation_node 기동 (StartRobotDriver 내부에서 호출).

        SDK 모드 / 토픽 모드를 settings 로부터 자동 분기. 빌트인 로봇은 launch
        패키지가 동일 namespace 안에서 controller 명령 토픽을 expose 하므로
        output_topic 은 leading '/' 만 제거한 상대 경로로 전달한다 — controller
        가 `<controller_name>/commands` 같은 하위 경로에 listen 해도 namespace
        밑에서 그대로 resolve 된다.
        """
        interp_id = self._interp_process_id(robot_id)
        self._stop(interp_id)

        if settings.get('sdk_control'):
            sdk_type = settings.get('sdk_type', '')
            can_port = settings.get('can_port', 'can0')
            # Linux CAN 인터페이스 이름은 underscore 없는 형태(canX)여야 한다.
            if isinstance(can_port, str) and can_port.startswith('can_'):
                can_port = 'can' + can_port[4:]
            has_gripper = rtype not in ('piper_no_gripper',)
            ip_address = settings.get('ip_address', '')
            serial_port = settings.get('serial_port', '')
            params = {
                'control_mode': 'sdk',
                'sdk_type': sdk_type,
                'sdk_can_port': can_port,
                'sdk_has_gripper': str(has_gripper).lower(),
                # ethernet 기반 SDK 로봇(Fairino 등)을 위해 ip_address 도 같이 전달.
                # interpolation_node 의 _build_interpolation_cmd 가 빈 문자열은
                # 자동으로 skip 하므로 CAN 전용 로봇(Piper)에서는 no-op.
                'sdk_ip_address': ip_address,
                # serial 기반 SDK 로봇(Robotiq 등)용 USB/RS485 포트 경로.
                'sdk_serial_port': serial_port,
                'read_topic': 'interpolated_joint_cmd',
            }
        else:
            # backend (robot_model) 이 write_topic 에 이미 `/ec_robot_<id>/`
            # prefix 를 붙여 보내므로 그대로 절대 경로로 사용한다 — ROS2 는 leading
            # `/` 가 있으면 node namespace 를 무시하므로 controller 가 nested 경로
            # (`<controller_name>/commands`) 를 쓰는 robot (omx, rbpodo, kinova) 도
            # 정상 매칭된다.
            write_topic = settings.get('write_topic', '/joint_states')
            params = {'output_topic': write_topic}
            # 컨트롤러가 std_msgs/Float64MultiArray 또는 trajectory_msgs/JointTrajectory
            # 를 받는 경우 module.json 의 write_topic_msg 를 그대로 노드 param 으로
            # 넘긴다. 미지정 시 interpolation_node default(sensor_msgs/JointState).
            msg_type = settings.get('write_topic_msg')
            if msg_type:
                params['output_msg_type'] = msg_type

        # module.json driver.interpolation_hz 가 명시되면 노드 publish_rate 로 전달.
        # 미지정 시 interpolation_node 의 ROS param default(200Hz)가 적용된다.
        hz = settings.get('interpolation_hz')
        if hz:
            params['publish_rate'] = float(hz)

        cmd = self._build_interpolation_cmd(robot_id, params)
        return self._start_subprocess(interp_id, cmd, log_name=log_name)

    def StartInterpolation(self, request, context):
        """Custom 로봇용 standalone 보간 노드 기동.

        custom robot 은 빌트인 처럼 launch 단계에서 자동 기동되지 않으므로
        RemoteAgent.__init__ 에서 (interpolation=True 일 때) 명시적으로 이 RPC
        를 호출한다. ec_joint_cmd → output_topic 으로 200Hz 보간 publish.
        StopInterpolation 으로 종료.
        """
        robot_id = request.robot_id
        if robot_id <= 0:
            return pb.DriverStatus(success=False, message='robot_id is required')

        output_topic = request.output_topic or ''
        msg_type = request.output_msg_type or 'sensor_msgs/JointState'

        params = {
            'output_topic': output_topic,
            'output_msg_type': msg_type,
            'control_mode': 'topic',
        }
        if request.publish_rate and request.publish_rate > 0:
            params['publish_rate'] = request.publish_rate

        interp_id = self._interp_process_id(robot_id)
        cmd = self._build_interpolation_cmd(robot_id, params)
        proc = self._start_subprocess(interp_id, cmd, log_name=interp_id)
        if proc is None:
            return pb.DriverStatus(success=False, message='Failed to start interpolation node')
        return pb.DriverStatus(success=True, message='Interpolation node started', pid=proc.pid)

    def StopInterpolation(self, request, context):
        """Stop the interpolation node started by StartInterpolation.

        idempotent — 없는 프로세스 이름이면 no-op."""
        self._stop(request.name)
        return pb.StatusResponse(success=True, message='Stopped')

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------
    def ListProcesses(self, request, context):
        with self._lock:
            return pb.ProcessList(names=list(self.processes.keys()))

    def ListTopics(self, request, context):
        all_topics = self.node.get_topic_names_and_types()
        topics = [
            pb.TopicInfo(name=name, type=types[0])
            for name, types in all_topics
            if self.node.count_publishers(name) > 0
        ]
        return pb.TopicList(topics=topics)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _build_robot_command(self, company, rtype, robot_id, settings):
        """robot driver 시작 argv. module.json driver.launch 만 본다 (manifest-driven)."""
        launch = get_robot_driver_launch(rtype)
        if not launch:
            return None
        ctx = _build_ctx('robot', robot_id, settings)
        return _build_argv_from_launch(launch, ctx)

    def _build_sensor_command(self, company, stype, sensor_id, settings):
        """sensor driver 시작 argv. module.json driver.launch 만 본다 (manifest-driven)."""
        launch = get_sensor_driver_launch(stype)
        if not launch:
            return None
        ctx = _build_ctx('sensor', sensor_id, settings)
        return _build_argv_from_launch(launch, ctx)

    # ------------------------------------------------------------------
    # Pre / post launch hooks (manifest 정의)
    # ------------------------------------------------------------------
    def _resolve_module_install_root(self, module_id: str, root_kind: str) -> str | None:
        """주어진 module_id 의 실제 install 폴더 경로를 디스크에서 찾는다.
        manifest 의 id 와 install 폴더 이름이 다를 수 있어 (예: id=robot_piper, 폴더=piper)
        ros2_ws/src/*/module.json 또는 robot_sdk/*/module.json 을 스캔."""
        if root_kind == 'sdk':
            base = '/root/robot_sdk'
        else:
            base = '/root/ros2_ws/src'
        if not os.path.isdir(base):
            return None
        try:
            for entry in os.listdir(base):
                d = os.path.join(base, entry)
                mj = os.path.join(d, 'module.json')
                if not os.path.isfile(mj):
                    continue
                try:
                    with open(mj) as f:
                        if json.load(f).get('id') == module_id:
                            return d
                except Exception:
                    continue
        except OSError:
            return None
        return None

    def _run_pre_launch_hook(self, hook: dict, robot_id: int, module_id: str,
                              ctx: dict, log_name: str) -> None:
        """driver.pre_launch 항목 하나 실행.

        지원 type:
          - "script": 주어진 path 의 bash 스크립트 실행. 보통 OS 셋업 (예: CAN bring-up).
            args: {type, path, [root="ros2"|"sdk"], [wait_after]}
            path 가 / 로 시작하면 absolute, 아니면 root 기준 상대:
              root="ros2" → ros2_ws/src/<module install dir>/<path>
              root="sdk"  → /root/robot_sdk/<module install dir>/<path>
            install dir 은 module_id 와 다를 수 있으므로(piper 등) 디스크 스캔으로 결정.
        """
        htype = (hook.get('type') or '').strip().lower()
        if htype == 'script':
            raw_path = hook.get('path', '').strip()
            if not raw_path:
                return
            substituted = _substitute_launch_args(raw_path, ctx)
            if substituted.startswith('/'):
                path = substituted
            else:
                root = (hook.get('root') or 'ros2').strip().lower()
                base = self._resolve_module_install_root(module_id, root)
                if base is None:
                    # fallback: module_id 를 그대로 폴더명으로 가정 (wizard 로 만든 모듈은 일치)
                    if root == 'sdk':
                        base = f'/root/robot_sdk/{module_id}'
                    else:
                        base = f'/root/ros2_ws/src/{module_id}'
                path = os.path.join(base, substituted)
            if not os.path.isfile(path):
                print(f"[pre_launch] script not found: {path} (module_id={module_id})", flush=True)
                return
            self._start_subprocess(f'pre_launch_{robot_id}_{os.path.basename(path)}',
                                   ['bash', path], log_name=log_name)
            wait = float(hook.get('wait_after', 1.0))
            if wait > 0:
                time.sleep(wait)
        else:
            print(f"[pre_launch] unsupported type: {htype}", flush=True)

    def _run_post_launch_hook(self, hook: dict, ctx: dict) -> None:
        """driver.post_launch 항목 하나 실행.

        지원 type:
          - "ros_service": ROS 2 서비스 호출. driver 노드가 떠 있어야 함.
            args: {type, service, service_type, request, [wait_before, timeout]}
        """
        htype = (hook.get('type') or '').strip().lower()
        if htype == 'ros_service':
            wait_before = float(hook.get('wait_before', 0.0))
            if wait_before > 0:
                time.sleep(wait_before)
            service = _substitute_launch_args(hook.get('service', ''), ctx)
            srv_type = hook.get('service_type', '')
            if not service or not srv_type:
                print(f"[post_launch] missing service/service_type", flush=True)
                return
            # service_type "pkg/srv/Name" → import pkg.srv (then attr Name)
            parts = srv_type.split('/')
            if len(parts) != 3:
                print(f"[post_launch] service_type 형식 'pkg/srv/Name' 이어야 함: {srv_type}", flush=True)
                return
            pkg, kind, name = parts
            try:
                mod = __import__(f'{pkg}.{kind}', fromlist=[name])
                cls = getattr(mod, name)
            except Exception as e:
                print(f"[post_launch] import {srv_type} failed: {e}", flush=True)
                return
            try:
                client = self.node.create_client(cls, service)
                timeout = float(hook.get('timeout', 5.0))
                if not client.wait_for_service(timeout_sec=timeout):
                    print(f"[post_launch] service {service} not available within {timeout}s", flush=True)
                    return
                req = cls.Request()
                for k, v in (hook.get('request') or {}).items():
                    if isinstance(v, str):
                        v = _substitute_launch_args(v, ctx)
                    setattr(req, k, v)
                client.call_async(req)
            except Exception as e:
                print(f"[post_launch] service call failed for {service}: {e}", flush=True)
        else:
            print(f"[post_launch] unsupported type: {htype}", flush=True)

    @staticmethod
    def _shell_quote(s):
        import shlex
        return shlex.quote(str(s))

    def _start_subprocess(self, name, command, log_name=None):
        log_name = log_name or name
        with self._lock:
            if name in self.processes:
                self._stop_locked(name)

            try:
                popen_args = {
                    'stdout': subprocess.PIPE, 'stderr': subprocess.PIPE,
                    'text': True, 'bufsize': 1,
                }
                if os.name != 'nt':
                    popen_args['start_new_session'] = True
                    command = ['stdbuf', '-oL'] + command

                proc = subprocess.Popen(command, **popen_args)
                self.processes[name] = proc

                # Background log reader
                def _log_reader(stream, label):
                    try:
                        for line in stream:
                            print(f"[{log_name}/{label}] {line.strip()}", flush=True)
                    except Exception:
                        pass

                threading.Thread(target=_log_reader, args=(proc.stdout, 'stdout'), daemon=True).start()
                threading.Thread(target=_log_reader, args=(proc.stderr, 'stderr'), daemon=True).start()

                # Wait for process end
                def _wait():
                    proc.wait()
                    with self._lock:
                        self.processes.pop(name, None)
                    print(f"Process '{name}' exited with code {proc.returncode}", flush=True)

                threading.Thread(target=_wait, daemon=True).start()
                return proc
            except Exception as e:
                print(f"[ERROR] Failed to start '{name}': {e}", flush=True)
                return None

    def _stop(self, name):
        with self._lock:
            self._stop_locked(name)

    def _stop_locked(self, name):
        proc = self.processes.pop(name, None)
        if proc is None:
            return
        try:
            if os.name != 'nt':
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        print(f"Process '{name}' terminated.", flush=True)

    def stop_all(self):
        with self._lock:
            for name in list(self.processes.keys()):
                self._stop_locked(name)
