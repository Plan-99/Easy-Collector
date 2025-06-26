import subprocess
import os
import signal
import atexit

class ProcessManager:
    def __init__(self, socketio):
        self.processes = {}
        self.socketio = socketio
        atexit.register(self.stop_all_processes)
        print("ProcessManager initialized and cleanup registered.")

    def list_processes(self):
        return [name for name, process in self.processes.items() if process.poll() is None]

    def _stream_reader(self, process_name, process_stream, stream_type, sid):
        """스트림을 안정적으로 읽어 클라이언트로 전송합니다."""
        try:
            for line in process_stream:
                log_data = {'log': line.strip(), 'type': stream_type}
                self.socketio.emit(f"log_{process_name}", log_data, to=sid)
        finally:
            process_stream.close()

    def start_process(self, name, command, sid=None):
        if name in self.processes and self.processes[name].poll() is None:
            print(f"[ERROR] '{name} Process' is already running.")
            return None

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

            self.socketio.start_background_task(target=self._stream_reader, process_name=name, process_stream=process.stdout, stream_type='stdout', sid=sid)
            self.socketio.start_background_task(target=self._stream_reader, process_name=name, process_stream=process.stderr, stream_type='stderr', sid=sid)

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