import subprocess
import os
import signal
import atexit
import traceback
import io
from contextlib import redirect_stdout
import sys

def get_log_type(message, default_type='stdout'):
    """메시지 내용을 분석하여 로그 타입을 반환합니다."""
    m = message.strip()
    if m.startswith("[ERROR]"): return 'error'
    if m.startswith("[NOTICE]"): return 'notice'
    if m.startswith("[SUCCESS]"): return 'success'
    if m.startswith("[WARNING]"): return 'warning'
    return default_type


class SocketIOStream(io.TextIOBase):
    def __init__(self, socketio, task_name):
        self.socketio = socketio
        self.task_name = task_name
        self.terminal = sys.__stdout__

    def write(self, s):
        # 공백이나 줄바꿈만 있는 경우 처리하지 않음
        stripped_s = s.strip()
        if stripped_s:
            self.terminal.write(stripped_s + '\n')
            self.terminal.flush()

        if not stripped_s:
            return len(s)

        msg_type = get_log_type(stripped_s)

        # 3. 설정된 type과 함께 emit
        self.socketio.emit('task_log', {
            'id': self.task_name, 
            'message': s, # 원본 문자열 전송 (또는 stripped_s)
            'type': msg_type
        })
        
        return len(s)

    def flush(self):
        self.terminal.flush()

class ProcessManager:
    def __init__(self, socketio, debug=False):
        self.processes = {}
        self.tasks = {}
        self.socketio = socketio
        self.debug = debug
        self.process_queue = {
            'train_task': [],
        }
        atexit.register(self.stop_all_processes)
        print("ProcessManager initialized and cleanup registered.")

    def list_processes(self):
        return [name for name, process in self.processes.items()]

    def _stream_reader(self, process_name, process_stream, log_emit_id, stream_type, sid):
        """스트림을 안정적으로 읽어 클라이언트로 전송합니다."""
        try:
            for line in process_stream:
                stripped_line = line.strip()
                msg_type = get_log_type(stripped_line)
                log_data = {
                    'id': log_emit_id, 
                    'message': stripped_line, 
                    'type': msg_type
                }
                self.socketio.emit('task_log', log_data, to=sid)
                if self.debug:
                    print(f"[{process_name} {stream_type}] {line.strip()}")
        finally:
            process_stream.close()

    def start_process(self, name, command, log_emit_id=None, sid=None):
        if name in self.processes and self.processes[name].poll() is None:
            print(f"[ERROR] '{name} Process' is already running.")
            return None
        
        if log_emit_id is None:
            log_emit_id = name

        command_full = ' '.join(command)

        # 1. 현재 파일(process_manager.py)의 폴더 경로: /root/src/backend/api
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 2. 프로젝트 루트 찾기 (두 단계 위로 올라가야 함)
        #    /root/src/backend/api  ->  /root/src/backend  ->  /root/src
        project_root = os.path.dirname(os.path.dirname(current_dir))

        # --- [디버깅 확인] ---
        # 이 로그가 출력될 때 '/root/src' 가 나오면 성공입니다.
        print(f"Calculated Project Root: {project_root}") 
        # ------------------

        current_env = os.environ.copy()
        current_env["PYTHONPATH"] = project_root + os.pathsep + current_env.get("PYTHONPATH", "")
        current_env["LIBGL_ALWAYS_SOFTWARE"] = "1"

        try:
            popen_args = {
                "stdout": subprocess.PIPE, 
                "stderr": subprocess.PIPE, 
                "text": True, 
                "bufsize": 1,
                "env": current_env,   # <--- 환경 변수 전달 (PYTHONPATH 포함됨)
                "cwd": project_root   # <--- 실행 위치 고정
            }

            if os.name != 'nt':
                popen_args["preexec_fn"] = os.setsid
                final_command = ['stdbuf', '-oL'] + command
            else:
                final_command = command

            process = subprocess.Popen(final_command, **popen_args)
            self.processes[name] = process  # 딕셔너리에 먼저 추가

            self.socketio.start_background_task(target=self._stream_reader, process_name=name, process_stream=process.stdout, log_emit_id=log_emit_id, stream_type='stdout', sid=sid)
            self.socketio.start_background_task(target=self._stream_reader, process_name=name, process_stream=process.stderr, log_emit_id=log_emit_id, stream_type='stderr', sid=sid)

            if name in self.process_queue:
                self.process_queue[name].pop(0)

            def wait_for_process_end():
                return_code = process.wait()
                print(f"Process '{name}' (PID: {process.pid}) has finished with return code {return_code}.")
                # 프로세스 종료 후 딕셔너리에서 제거
                if name in self.processes:
                    del self.processes[name]

                self.socketio.emit(f"stop_process", {'id': name, 'return_code': return_code}, to=sid)

                if name in self.process_queue and len(self.process_queue[name]) > 0:
                    next_command = self.process_queue[name][0]['command']
                    print(f"Starting next queued process for '{name}': {' '.join(next_command)}")
                    self.start_process(name, next_command, log_emit_id=log_emit_id, sid=sid)
                


            self.socketio.start_background_task(target=wait_for_process_end)
            print(f"'{name}' Process started. PID: {process.pid} / Command: {command_full}")
            self.socketio.emit(f"start_process", { 'id': name }, to=sid)

            return process
        except Exception as e:
            print(f"[ERROR]: {e}")
            return None

    def stop_process(self, name):
        if name not in self.processes:
            print(f"'{name}' Process is not running.")
            return
        process = self.processes.pop(name)
        if process.poll() is None:
            print(f"'{name}' Terminating process(PID: {process.pid})")
            try:
                if os.name != 'nt':
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                else:
                    os.kill(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            except Exception:
                process.kill()
            print(f"'{name}' Process terminated.")

    def start_function(self, name, func, log_id=None, *args, **kwargs):
        if name in self.tasks and self.tasks[name]['stop'] is False:
            print(f"[ERROR] '{name}' Function is already running.", self.tasks)
            return
        
        if log_id is None:
            log_id = name

        # 1. 작업을 제어할 '중지 플래그'를 생성합니다.
        task_control = {'stop': False}
        self.tasks[name] = task_control
        self.processes[name] = 'function'  # 프로세스 딕셔너리에 추가
        
        # 2. **kwargs에 제어 플래그를 추가하여 target 함수로 전달합니다.
        kwargs['task_control'] = task_control

        def task_wrapper(*wrapper_args, **wrapper_kwargs):
            print(f"Wrapper for '{name}' started.")
            # 소켓 스트림 생성
            socket_stream = SocketIOStream(self.socketio, log_id)
            
            try:
                # redirect_stdout을 사용하여 이 블록 안의 print를 가로챕니다.
                with redirect_stdout(socket_stream):
                    func(*wrapper_args, **wrapper_kwargs)
            except Exception as e:
                error_traceback = traceback.format_exc()
                print(f"[ERROR] {error_traceback}")
                self.socketio.emit('task_log', {'id': log_id, 'message': f"Error: {str(e)}"})
            finally:
                print(f"Task '{name}' finished. Cleaning up.")
                if name in self.tasks:
                    del self.tasks[name]
                self.socketio.emit('stop_process', {'id': name})

        try:
            # 4. 래퍼 함수에 *args와 **kwargs를 올바르게 전달합니다.
            self.socketio.start_background_task(target=task_wrapper, *args, **kwargs)
            self.socketio.emit('start_process', {'id': name})
            print(f"Task '{name}' has been started in the background.")
        except Exception as e:
            print(f"[ERROR] Failed to start task '{name}': {e}")
            if name in self.tasks:
                del self.tasks[name]
        
        
    def stop_function(self, name):
        if name not in self.tasks:
            print(f"'{name}' Function is not running.")
            return
        
        if name in self.processes:
            del self.processes[name]  # 프로세스 딕셔너리에 추가

        # 4. 실제 스레드를 종료하는 대신, '중지 플래그'를 True로 바꿉니다.
        task_control = self.tasks.get(name)
        self.socketio.emit(f"stop_process", {'id': name})
        if task_control:
            print(f"'{name}' Function stopping.")
            task_control['stop'] = True

        
        
    def stop_all_processes(self):
        print("--- stop_all_processes called ---")
        # 이 시점에 self.processes가 비어있는지 확인
        print(f"Current processes: {self.processes}")
        if not self.processes:
            print("No processes to stop.")
            return
            
        for name in list(self.processes.keys()):
            if self.processes.get(name) == 'function':
                self.stop_function(name)
            else:
                self.stop_process(name)
        print("All processes terminated.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_all_processes()