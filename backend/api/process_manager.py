"""
ProcessManager — 백엔드의 백그라운드 작업(서브프로세스 + 함수 워커) 통합 레지스트리.

내부 구조는 Task 다형(`FunctionTask` / `SubprocessTask`)으로 분리되어 있지만,
외부 인터페이스는 기존 코드와 100% 호환:
  - `pm.processes`             : `dict[str, dict]` (entry = {'type','obj','sid'?})
  - `pm.list_processes()`      : `list[str]`
  - `pm.start_function(name, func, log_id=None, *args, **kwargs)`
  - `pm.stop_function(name)`   : 비동기 — task_control['stop']=True 세팅 후 즉시 리턴
  - `pm.start_process(name, command, log_emit_id=None, sid=None)`
  - `pm.stop_process(name)`    : 동기 — SIGTERM → wait → SIGKILL
  - `pm.stop_all_processes()`

`task_control`은 함수 워커에 자동 주입되는 shared mutable dict — 워커와 호출자
양쪽이 'stop', 'episode_stop', 'succeed' 등 키를 read/write 한다 (record_episode,
leader_teleoperation, remote_train_workflow 등이 의존).

stop_function의 `socketio.emit('stop_process', ...)` 호출은 task_wrapper.finally가
단일 책임으로 발행하도록 일원화되어, stop 시그널과 워커 종료 사이의 동시 emit
race가 줄어든다 (2026-04 SIGSEGV 방지). UI 입장에서는 stop 이벤트가 워커가 실제로
loop를 빠져나간 시점(보통 ~100ms 후)에 도착하는 점만 다른데, 의미상 더 정확.
"""
import atexit
import io
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
from contextlib import redirect_stdout
from typing import Any, Callable, Optional


def get_log_type(message, default_type='stdout'):
    """메시지 내용을 분석하여 로그 타입을 반환합니다."""
    m = message.strip()
    if m.startswith("[ERROR]"): return 'error'
    if m.startswith("[NOTICE]"): return 'notice'
    if m.startswith("[SUCCESS]"): return 'success'
    if m.startswith("[WARNING]"): return 'warning'
    return default_type


class SocketIOStream(io.TextIOBase):
    """함수 워커의 stdout을 socketio task_log 이벤트로 전달."""

    def __init__(self, socketio, task_name):
        self.socketio = socketio
        self.task_name = task_name
        self.terminal = sys.__stdout__

    def write(self, s):
        stripped_s = s.strip()
        if stripped_s:
            self.terminal.write(stripped_s + '\n')
            self.terminal.flush()
            msg_type = get_log_type(stripped_s)
            self.socketio.emit('task_log', {
                'id': self.task_name,
                'message': s,
                'type': msg_type,
            })
        return len(s)

    def flush(self):
        self.terminal.flush()


# ---------------------------------------------------------------------------
# Internal Task abstractions — pm.processes는 dict facade로만 노출.
# ---------------------------------------------------------------------------

class _Task:
    """Common base. type 필드는 외부 호환성 위해 string discriminant로 노출."""
    type: str = ''
    name: str = ''

    def to_dict(self) -> dict:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError


class FunctionTask(_Task):
    """소켓IO 백그라운드 태스크로 돌아가는 함수 워커. 협력적(cooperative) 종료."""
    type = 'function'

    def __init__(self, name: str, control: dict):
        self.name = name
        self.control = control  # task_control: shared mutable dict

    def to_dict(self) -> dict:
        # 기존 코드가 `pm.processes[name]['obj']['stop'] = True` 식으로
        # 직접 dict mutation 했으므로, 'obj'는 control dict 그대로 노출.
        return {'type': 'function', 'obj': self.control}

    def stop(self) -> None:
        # 협력적 종료 — 워커가 다음 loop iteration에서 자기 자신을 정리한다.
        # task_wrapper.finally가 레지스트리 제거 + socketio.emit 담당.
        self.control['stop'] = True


class SubprocessTask(_Task):
    """별도 프로세스로 실행되는 서브프로세스 워커. 동기적 강제 종료."""
    type = 'subprocess'

    def __init__(self, name: str, process: subprocess.Popen, sid: Optional[str] = None):
        self.name = name
        self.process = process
        self.sid = sid

    def to_dict(self) -> dict:
        d = {'type': 'subprocess', 'obj': self.process}
        if self.sid is not None:
            d['sid'] = self.sid
        return d

    def stop(self, timeout: float = 5.0) -> None:
        proc = self.process
        try:
            if os.name != 'nt':
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=timeout)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# ProcessManager
# ---------------------------------------------------------------------------

class ProcessManager:
    def __init__(self, socketio, debug=False):
        self._lock = threading.RLock()
        self._tasks: dict[str, _Task] = {}
        self.socketio = socketio
        self.debug = debug
        # legacy: subprocess 종료 시 큐에 대기 중인 다음 명령을 자동 시작하는 훅.
        # 현재는 train_task 키만 정의돼 있지만 실 사용 코드가 사라져 dead-ish.
        # 호환성 위해 유지 (외부에서 list mutation할 가능성).
        self.process_queue = {'train_task': []}

        atexit.register(self.stop_all_processes)
        print("ProcessManager initialized (Unified Registry Mode).")

    # ------------------------------------------------------------------
    # 호환성 facade — 기존 코드가 pm.processes를 dict-like로 접근하던 패턴 보존.
    # ------------------------------------------------------------------
    @property
    def processes(self) -> dict:
        with self._lock:
            return {name: t.to_dict() for name, t in self._tasks.items()}

    def list_processes(self) -> list:
        with self._lock:
            return list(self._tasks.keys())

    def _register(self, name: str, task: _Task) -> bool:
        """이름이 비어 있으면 등록하고 True. 이미 있으면 False."""
        with self._lock:
            if name in self._tasks:
                return False
            self._tasks[name] = task
            return True

    def _unregister(self, name: str) -> Optional[_Task]:
        with self._lock:
            return self._tasks.pop(name, None)

    def _get(self, name: str) -> Optional[_Task]:
        with self._lock:
            return self._tasks.get(name)

    # ------------------------------------------------------------------
    # Subprocess
    # ------------------------------------------------------------------
    def _stream_reader(self, name, process_stream, log_emit_id, stream_type, sid):
        try:
            for line in process_stream:
                stripped_line = line.strip()
                if not stripped_line:
                    continue
                msg_type = get_log_type(stripped_line)
                self.socketio.emit('task_log', {
                    'id': log_emit_id,
                    'message': stripped_line,
                    'type': msg_type,
                }, to=sid)
                if self.debug:
                    print(f"[{name} {stream_type}] {stripped_line}")
        finally:
            process_stream.close()

    def start_process(self, name, command, log_emit_id=None, sid=None):
        # 사전 중복 체크 — race를 줄이기 위해 _register에서도 한 번 더 체크
        if self._get(name) is not None:
            print(f"[ERROR] '{name}' is already running.")
            return None

        log_emit_id = log_emit_id or name
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(current_dir))

        current_env = os.environ.copy()
        current_env["PYTHONPATH"] = project_root + os.pathsep + current_env.get("PYTHONPATH", "")
        current_env["LIBGL_ALWAYS_SOFTWARE"] = "1"

        try:
            popen_args = {
                "stdout": subprocess.PIPE, "stderr": subprocess.PIPE,
                "text": True, "bufsize": 1, "env": current_env, "cwd": project_root,
            }
            if os.name != 'nt':
                popen_args["preexec_fn"] = os.setsid
                command = ['stdbuf', '-oL'] + command

            process = subprocess.Popen(command, **popen_args)
            task = SubprocessTask(name=name, process=process, sid=sid)
            if not self._register(name, task):
                # 동시 호출 race — 우리가 띄운 새 process는 즉시 정리.
                print(f"[ERROR] '{name}' is already running (race).")
                try:
                    if os.name != 'nt':
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    else:
                        process.terminate()
                except Exception:
                    pass
                return None

            self.socketio.start_background_task(
                target=self._stream_reader, name=name, process_stream=process.stdout,
                log_emit_id=log_emit_id, stream_type='stdout', sid=sid,
            )
            self.socketio.start_background_task(
                target=self._stream_reader, name=name, process_stream=process.stderr,
                log_emit_id=log_emit_id, stream_type='stderr', sid=sid,
            )

            def wait_for_process_end():
                return_code = process.wait()
                print(f"Process '{name}' (PID: {process.pid}) finished with code {return_code}.")
                self._unregister(name)
                self.socketio.emit("stop_process", {'id': name, 'return_code': return_code}, to=sid)

                # 큐 처리 (legacy)
                with self._lock:
                    queued = self.process_queue.get(name) or []
                    next_task = queued.pop(0) if queued else None
                if next_task is not None:
                    self.start_process(name, next_task['command'], log_emit_id=log_emit_id, sid=sid)

            self.socketio.start_background_task(target=wait_for_process_end)
            self.socketio.emit("start_process", {'id': name}, to=sid)
            return process
        except Exception as e:
            print(f"[ERROR]: {e}")
            self._unregister(name)
            return None

    def stop_process(self, name):
        task = self._get(name)
        if task is None or not isinstance(task, SubprocessTask):
            print(f"'{name}' Process is not running.")
            return
        task.stop()
        # wait_for_process_end가 이미 _unregister + emit('stop_process')를 처리하지만,
        # 동기 종료를 보장하기 위해 여기서도 안전하게 한 번 더 보장.
        self._unregister(name)
        print(f"'{name}' Process terminated.")

    # ------------------------------------------------------------------
    # Function
    # ------------------------------------------------------------------
    def start_function(self, name, func, log_id=None, *args, **kwargs):
        if self._get(name) is not None:
            print(f"[ERROR] '{name}' is already running.")
            return

        log_id = log_id or name
        # task_control은 워커에 kwarg로 전달되는 shared mutable dict.
        # 외부에서도 pm.processes[name]['obj']로 접근 가능 (호환성).
        control: dict = {'stop': False}
        task = FunctionTask(name=name, control=control)
        if not self._register(name, task):
            print(f"[ERROR] '{name}' is already running (race).")
            return

        kwargs['task_control'] = control

        def task_wrapper(*wrapper_args, **wrapper_kwargs):
            socket_stream = SocketIOStream(self.socketio, log_id)
            try:
                with redirect_stdout(socket_stream):
                    func(*wrapper_args, **wrapper_kwargs)
            except Exception as e:
                print(f"[ERROR] {traceback.format_exc()}")
                self.socketio.emit('task_log', {'id': log_id, 'message': f"Error: {str(e)}"})
            finally:
                # 종료 알림은 워커가 실제로 빠져나온 시점에서 한 번만 발행 —
                # stop_function이 동시에 emit해서 race를 만드는 일이 없도록.
                self._unregister(name)
                self.socketio.emit('stop_process', {'id': name})

        self.socketio.start_background_task(target=task_wrapper, *args, **kwargs)
        self.socketio.emit('start_process', {'id': name})

    def stop_function(self, name, wait_timeout: Optional[float] = None):
        """협력적 종료 신호 — task_control['stop']=True 세팅 후 즉시 리턴.

        wait_timeout이 주어지면 워커가 실제로 정리될 때까지 폴링으로 대기 (선택).
        호환성: 기존 호출자는 wait_timeout 없이 부르며 동작은 동일하다.
        """
        task = self._get(name)
        if task is None or not isinstance(task, FunctionTask):
            print(f"'{name}' Function is not running.")
            return

        task.stop()
        print(f"'{name}' Function stop signal sent.")

        if wait_timeout and wait_timeout > 0:
            deadline = time.monotonic() + wait_timeout
            while time.monotonic() < deadline:
                if self._get(name) is None:
                    return
                time.sleep(0.05)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def stop_all_processes(self):
        """기존 명칭 유지: 모든 타입의 프로세스/함수를 종료합니다."""
        print("--- stop_all_processes called ---")
        with self._lock:
            names = list(self._tasks.keys())
        for name in names:
            task = self._get(name)
            if task is None:
                continue
            try:
                if isinstance(task, SubprocessTask):
                    self.stop_process(name)
                elif isinstance(task, FunctionTask):
                    self.stop_function(name)
            except Exception as e:
                print(f"[ERROR] stopping '{name}': {e}")
        print("All processes/functions terminated.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_all_processes()
