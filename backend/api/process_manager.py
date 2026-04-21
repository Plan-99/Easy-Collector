import subprocess
import os
import signal
import atexit
import traceback
import io
import sys
from contextlib import redirect_stdout

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
        stripped_s = s.strip()
        if stripped_s:
            self.terminal.write(stripped_s + '\n')
            self.terminal.flush()
            msg_type = get_log_type(stripped_s)
            self.socketio.emit('task_log', {
                'id': self.task_name, 
                'message': s, 
                'type': msg_type
            })
        return len(s)

    def flush(self):
        self.terminal.flush()

class ProcessManager:
    def __init__(self, socketio, debug=False):
        # self.tasks를 제거하고 self.processes 하나로 통합 관리합니다.
        self.processes = {} 
        self.socketio = socketio
        self.debug = debug
        self.process_queue = {'train_task': []}
        
        atexit.register(self.stop_all_processes)
        print("ProcessManager initialized (Unified Registry Mode).")

    def list_processes(self):
        return list(self.processes.keys())

    def _stream_reader(self, name, process_stream, log_emit_id, stream_type, sid):
        try:
            for line in process_stream:
                stripped_line = line.strip()
                if not stripped_line: continue
                
                msg_type = get_log_type(stripped_line)
                self.socketio.emit('task_log', {
                    'id': log_emit_id, 
                    'message': stripped_line, 
                    'type': msg_type
                }, to=sid)
                
                if self.debug:
                    print(f"[{name} {stream_type}] {stripped_line}")
        finally:
            process_stream.close()

    def start_process(self, name, command, log_emit_id=None, sid=None):
        if name in self.processes:
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
                "text": True, "bufsize": 1, "env": current_env, "cwd": project_root
            }
            if os.name != 'nt':
                popen_args["preexec_fn"] = os.setsid
                command = ['stdbuf', '-oL'] + command

            process = subprocess.Popen(command, **popen_args)

            # [통합 저장] 'type'을 명시하여 프로세스임을 저장
            self.processes[name] = {
                'type': 'subprocess',
                'obj': process,
                'sid': sid
            }

            self.socketio.start_background_task(target=self._stream_reader, name=name, process_stream=process.stdout, log_emit_id=log_emit_id, stream_type='stdout', sid=sid)
            self.socketio.start_background_task(target=self._stream_reader, name=name, process_stream=process.stderr, log_emit_id=log_emit_id, stream_type='stderr', sid=sid)

            def wait_for_process_end():
                return_code = process.wait()
                print(f"Process '{name}' (PID: {process.pid}) finished with code {return_code}.")
                if name in self.processes:
                    del self.processes[name]

                self.socketio.emit("stop_process", {'id': name, 'return_code': return_code}, to=sid)
                
                # 큐 처리 로직
                if name in self.process_queue and len(self.process_queue[name]) > 0:
                    next_task = self.process_queue[name].pop(0)
                    self.start_process(name, next_task['command'], log_emit_id=log_emit_id, sid=sid)

            self.socketio.start_background_task(target=wait_for_process_end)
            self.socketio.emit("start_process", {'id': name}, to=sid)
            return process
        except Exception as e:
            print(f"[ERROR]: {e}")
            return None

    def stop_process(self, name):
        task = self.processes.get(name)
        if not task or task['type'] != 'subprocess':
            print(f"'{name}' Process is not running.")
            return

        process = task['obj']
        try:
            if os.name != 'nt':
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            else:
                process.terminate()
            process.wait(timeout=5)
        except:
            process.kill()
        
        if name in self.processes:
            del self.processes[name]
        print(f"'{name}' Process terminated.")

    def start_function(self, name, func, log_id=None, *args, **kwargs):
        if name in self.processes:
            print(f"[ERROR] '{name}' is already running.")
            return
        
        log_id = log_id or name
        stop_flag = {'stop': False}
        
        self.processes[name] = {
            'type': 'function',
            'obj': stop_flag,
        }
        
        # --- 핵심 수정 부분 ---
        kwargs['task_control'] = stop_flag
        # ----------------------

        def task_wrapper(*wrapper_args, **wrapper_kwargs):
            socket_stream = SocketIOStream(self.socketio, log_id)
            try:
                with redirect_stdout(socket_stream):
                    # 여기서 wrapper_kwargs(즉, 수정된 kwargs)가 전달됩니다.
                    func(*wrapper_args, **wrapper_kwargs)
            except Exception as e:
                print(f"[ERROR] {traceback.format_exc()}")
                self.socketio.emit('task_log', {'id': log_id, 'message': f"Error: {str(e)}"})
            finally:
                if name in self.processes:
                    del self.processes[name]
                self.socketio.emit('stop_process', {'id': name})

        self.socketio.start_background_task(target=task_wrapper, *args, **kwargs)
        self.socketio.emit('start_process', {'id': name})

    def stop_function(self, name):
        task = self.processes.get(name)
        if not task or task['type'] != 'function':
            print(f"'{name}' Function is not running.")
            return
        
        # 플래그를 True로 변경하여 함수가 스스로 멈추게 함
        task['obj']['stop'] = True
        del self.processes[name]  # 삭제는 task_wrapper의 finally에서 수행하도록 변경
        self.socketio.emit('stop_process', {'id': name})
        print(f"'{name}' Function stop signal sent.")

    def stop_all_processes(self):
        """기존 명칭 유지: 모든 타입의 프로세스/함수를 종료합니다."""
        print("--- stop_all_processes called ---")
        for name in list(self.processes.keys()):
            task = self.processes.get(name)
            if not task: continue
            
            if task['type'] == 'subprocess':
                self.stop_process(name)
            elif task['type'] == 'function':
                self.stop_function(name)
        print("All processes/functions terminated.")

    def __enter__(self): return self
    def __exit__(self, exc_type, exc_val, exc_tb): self.stop_all_processes()