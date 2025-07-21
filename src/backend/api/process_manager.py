import subprocess
import os
import signal
import atexit

class ProcessManager:
    def __init__(self, socketio, debug=False):
        self.processes = {}
        self.tasks = {}
        self.socketio = socketio
        self.debug = debug
        atexit.register(self.stop_all_processes)
        print("ProcessManager initialized and cleanup registered.")

    def list_processes(self):
        return [name for name, process in self.processes.items() if process.poll() is None]

    def _stream_reader(self, process_name, process_stream, log_emit_id, stream_type, sid):
        """스트림을 안정적으로 읽어 클라이언트로 전송합니다."""
        try:
            for line in process_stream:
                log_data = {'log': line.strip(), 'type': stream_type}
                self.socketio.emit(log_emit_id, log_data, to=sid)
                if self.debug:
                    print(f"[{process_name} {stream_type}] {line.strip()}")
        finally:
            process_stream.close()

    def start_process(self, name, command, log_emit_id=None, sid=None):
        if name in self.processes and self.processes[name].poll() is None:
            print(f"[ERROR] '{name} Process' is already running.")
            return None
        
        if log_emit_id is None:
            log_emit_id = 'log_' + name

        command_full = ' '.join(command)
        try:
            popen_args = {"stdout": subprocess.PIPE, "stderr": subprocess.PIPE, "text": True, "bufsize": 1}
            if os.name != 'nt':
                popen_args["preexec_fn"] = os.setsid
                final_command = ['stdbuf', '-oL'] + command
            else:
                final_command = command

            process = subprocess.Popen(final_command, **popen_args)
            self.processes[name] = process  # 딕셔너리에 먼저 추가

            self.socketio.start_background_task(target=self._stream_reader, process_name=name, process_stream=process.stdout, log_emit_id=log_emit_id, stream_type='stdout', sid=sid)
            self.socketio.start_background_task(target=self._stream_reader, process_name=name, process_stream=process.stderr, log_emit_id=log_emit_id, stream_type='stderr', sid=sid)

            def wait_for_process_end():
                return_code = process.wait()
                print(f"Process '{name}' (PID: {process.pid}) has finished with return code {return_code}.")
                # 프로세스 종료 후 딕셔너리에서 제거
                if name in self.processes:
                    del self.processes[name]
                self.socketio.emit(f"stop_process", {'id': name, 'return_code': return_code}, to=sid)


            self.socketio.start_background_task(target=wait_for_process_end)
            print(f"'{name}' Process started. PID: {process.pid} / Command: {command_full}")
            self.socketio.emit(f"start_process", { 'id': name }, to=sid)

            return process
        except Exception as e:
            print(f"[ERROR]: {e}")
            return None

    def stop_process(self, name):
        if name not in self.processes:
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

    def start_function(self, name, func, *args, **kwargs):
        if name in self.tasks and self.tasks[name]['stop'] is False:
            print(f"[ERROR] '{name}' Function is already running.", self.tasks)
            return

        # 1. 작업을 제어할 '중지 플래그'를 생성합니다.
        task_control = {'stop': False}
        self.tasks[name] = task_control
        
        # 2. **kwargs에 제어 플래그를 추가하여 target 함수로 전달합니다.
        kwargs['task_control'] = task_control
        try:
            # 3. start_background_task의 반환값은 저장할 필요가 없습니다.
            self.socketio.start_background_task(target=func, *args, **kwargs)
            print(f"'{name}' Function started.")
        except Exception as e:
            print(f"[ERROR]: {e}")
            # 실패 시 딕셔너리에서 제거
            if name in self.tasks:
                del self.tasks[name]
        
        
    def stop_function(self, name):
        if name not in self.tasks:
            print(f"'{name}' Function is not running.")
            return

        # 4. 실제 스레드를 종료하는 대신, '중지 플래그'를 True로 바꿉니다.
        task_control = self.tasks.get(name)
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
            self.stop_process(name)
        print("All processes terminated.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_all_processes()