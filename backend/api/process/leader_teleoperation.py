# #!/usr/bin/env python

from concurrent.futures import thread
import math
import time
import threading
from ...env.dxl_controller import DxlController
from .subscribe_dynamixel import get_available_ports
from collections import defaultdict
import sys

from concurrent.futures import ThreadPoolExecutor


class _SimpleRate:
    """rclpy.Rate 대체. 지정 Hz로 sleep."""
    def __init__(self, hz):
        self._interval = 1.0 / hz
        self._last = time.monotonic()

    def sleep(self):
        now = time.monotonic()
        elapsed = now - self._last
        remaining = self._interval - elapsed
        if remaining > 0:
            time.sleep(remaining)
        self._last = time.monotonic()

class Leader():
    def __init__(self, node, agents, socketio_instance, teleop_setting) -> None:
        # ROS 노드 초기화ur5e/ur5e_scaled_pos_joint_traj_controller/command
        self.node = node
        self.socketio_instance = socketio_instance
        # self.origin = leader_robot_preset['origin']  # 다이나믹셀의 원점 위치
        # self.gripper_dxl_range = leader_robot_preset['gripper_dxl_range']  # 다이나믹셀의 원점 위치
        # self.sign_corrector = leader_robot_preset['sign_corrector']
        # self.dxl_ids = leader_robot_preset['dxl_ids']  # 다이나믹셀 ID
        # self.gripper_dxl_ids = leader_robot_preset.get('gripper_dxl_ids', [])
        # self.ema = float(leader_robot_preset.get('ema', 0.0)) # EMA 필터 값

        # # 트리거 그리퍼 설정 (기본값: 첫 번째 그리퍼)
        # self.trigger_gripper_index = 0
        # self.trigger_dxl_id = self.gripper_dxl_ids[self.trigger_gripper_index]

        self.address = 132  # 다이나믹셀의 현재 위치 주소 (주소 132번은 현재 위치)
        self.p_gain = 1
        self.is_synced = False
        self.agents = agents

        self.grouped_by_port = defaultdict(list)
        self.joint_map = teleop_setting['joint_map']

        # 각 에이전트에 맞게 joint_map 업데이트
        self.update_joint_map_with_agent_info()

        grouped_by_port = self.group_joints_by_port()
        self.dxl_controllers = {}
        for port, joints in grouped_by_port.items():
            self.dxl_controllers[port] = DxlController(
                serial_port=port,
                dxl_ids=[item['dxl_id'] for item in joints],
                gripper_dxl_ids=[item['dxl_id'] for item in joints if item.get('is_gripper', False)],
            )

        self.is_paused = [False] * len(self.dxl_controllers)
        self.is_cleaned_up = False
        

        self.ema = float(teleop_setting.get('ema', 0.2)) # EMA 필터 값 (낮을수록 응답성↑)
        # 한 스텝에 허용되는 최대 관절 변위(rad). 사실상 비상 안전 한도.
        # 보간 노드가 200Hz로 부드럽게 처리하므로 leader 쪽에서 누적 클리핑은 lag만 만든다.
        # 0.3 rad ≈ 17° 점프는 사람이 한 step에 도달 불가능 → 이상값/노이즈만 차단.
        self.max_step_rad = float(teleop_setting.get('max_step_rad', 0.3))
        # 텔레옵 publish rate (Hz). 보간 노드가 200Hz로 직접 제어하므로 50~200 사이 권장.
        self.publish_rate = float(teleop_setting.get('publish_rate', 100.0))

        self.thread_pool = ThreadPoolExecutor(max_workers=len(self.agents))



    def update_joint_map_with_agent_info(self):
        for joint_info in self.joint_map:
            # 구버전 preset 호환: agent_origin이 없으면 0 (≡ 팔로워 home 자세 기준 캘리브레이션).
            if 'agent_origin' not in joint_info or joint_info.get('agent_origin') is None:
                joint_info['agent_origin'] = 0.0
            for agent in self.agents:
                try:
                    if 'robot_id' in joint_info and agent.id == joint_info['robot_id']:
                        j_idx = agent.joint_names.index(joint_info['joint_name'])
                        joint_info['joint_upper_bound'] = agent.joint_upper_bounds[j_idx]
                        joint_info['joint_lower_bound'] = agent.joint_lower_bounds[j_idx]
                except ValueError:
                    continue


    def read_dxl_and_write_to_joint_map(self):
        group_by_port = self.group_joints_by_port()
        for port, joints in group_by_port.items():
            dxl_controller = self.dxl_controllers[port]
            positions = dxl_controller.read_all_dynamixel()
            for position in positions:
                joint = self.get_joint_by_dxl_id(port, position['id'])
                if joint is not None:
                    joint['dxl_position'] = position['position']

    def read_agent_and_write_to_joint_map(self):
        for agent in self.agents:
            joint_positions = agent.get_joint_states()
            for joint in self.joint_map:
                if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                    continue
                if joint['robot_id'] != agent.id:
                    continue
                joint_name = joint['joint_name']
                joint_index = agent.joint_names.index(joint_name)
                joint['agent_position'] = joint_positions[joint_index]


    def group_joints_by_port(self):
        grouped_by_port = defaultdict(list)
        for joint in self.joint_map:
            grouped_by_port[joint['port']].append(joint)

        return grouped_by_port
    
    def group_joints_by_agent(self):
        grouped_by_port = defaultdict(list)
        for joint in self.joint_map:
            if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                continue
            grouped_by_port[joint['robot_id']].append(joint)

        return grouped_by_port
    
    def get_joint_by_joint_name(self, agent, joint_name):
        for joint in self.joint_map:
            if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                continue
            if joint['robot_id'] != agent.id:
                continue
            if joint['joint_name'] == joint_name:
                return joint
        return None
    
    def get_joint_by_dxl_id(self, port, dxl_id):
        for joint in self.joint_map:
            if joint['port'] != port:
                continue
            if joint['dxl_id'] == dxl_id:
                return joint
        return None


    def get_rad_pos(self, joint):
        # 캘리브레이션 시점에 저장된 (origin, agent_origin):
        #   "리더 dxl이 origin일 때, 팔로워 관절은 agent_origin rad 자세였다"
        # 따라서 현재 리더 dxl_position은 origin 대비 delta_ticks → delta_rad 변환 후
        # agent_origin을 더해 팔로워 좌표계로 매핑한다.
        # pos = math.fmod((position - self.origin[dxl_id]), 4096)
        pos = (joint['dxl_position'] - joint['origin']) % 4096
        pos = pos / 4096 * 360
        if pos > 180:
            pos -= 360
        elif pos < -180:
            pos += 360
        delta_rad = pos / 360 * 2 * math.pi * joint['sign']
        return joint.get('agent_origin', 0.0) + delta_rad


    def rad_to_tick(self, rad, origin, sign, agent_origin=0.0):
        # get_rad_pos의 정확한 역함수.
        # delta_rad = rad - agent_origin  (팔로워 좌표 → 리더 좌표 차이)
        delta_rad = rad - agent_origin
        pos_deg = (delta_rad * 180.0 / math.pi * sign) % 360.0
        # Python의 % 연산자는 양수 divisor에 대해 항상 [0, divisor) 반환하므로
        # 추가 음수 보정은 불필요. 안전망으로 한번 더 클램프.
        if pos_deg < 0:
            pos_deg += 360
        pos_steps = pos_deg / 360.0 * 4096.0
        position = (pos_steps + origin) % 4096
        return int(round(position))
    

    def sync_leader_robot(self, task_control=None):
        self.is_synced = False
        self._sync_done_ports = set()
        self._sync_failed_ports = set()
        self._torque_release_pending_ports = set(self.dxl_controllers.keys())
        # 시작 시점에 그리퍼가 이미 닫혀 있으면 토크를 그대로 풀어버리지 않고
        # "한 번 열었다 닫기" 동작을 요구한다 (사람이 의식적으로 제어권을 가져가도록).
        # 포트별로 사용자가 그리퍼를 연 적이 있는지 추적.
        self._gripper_open_observed = {port: False for port in self.dxl_controllers}
        self._gripper_initial_close_logged = {port: False for port in self.dxl_controllers}
        self.read_agent_and_write_to_joint_map()

        groups_by_port = self.group_joints_by_port()

        def sync_dxl_controller(port, dxl_joints):
            try:
                dxl_controller = self.dxl_controllers[port]
                for dxl_joint in dxl_joints:
                    if 'is_dummy_gripper' in dxl_joint and dxl_joint['is_dummy_gripper']:
                        continue
                    dxl_id = dxl_joint['dxl_id']
                    agent_position = dxl_joint['agent_position']
                    pos = self.rad_to_tick(
                        agent_position,
                        dxl_joint['origin'],
                        dxl_joint['sign'],
                        dxl_joint.get('agent_origin', 0.0),
                    )
                    dxl_joint['dxl_goal_position'] = pos

                print("Syncing Leader Robot")
                moved = dxl_controller.move_controller(dxl_joints)
                if not moved:
                    raise Exception("move_controller returned False")

                # 홈포즈 도달 후 1초 뒤 sync 완료 처리 → position_pub 즉시 시작.
                # 토크 해제는 position_pub 루프에서 그리퍼 닫힘 감지 시 수행 (포트 동시 접근 회피).
                self._sync_done_ports.add(port)
                print("[NOTICE] Sync done. Waiting for gripper open/close gesture to release leader torque.")
            except Exception as e:
                # 실패 시에도 _sync_failed_ports에 등록해서 메인 루프가 무한 대기에 빠지지 않도록.
                print(f"[ERROR] sync_dxl_controller failed for port {port}: {e}")
                self._sync_failed_ports.add(port)

        for port, joints in groups_by_port.items():
            self.socketio_instance.start_background_task(
                target=sync_dxl_controller,
                port=port,
                dxl_joints=joints
            )

        ## 모든 컨트롤러가 홈포즈 이동 + 1초 대기를 마칠 때까지 대기.
        ## stop 신호 또는 에러 발생 시 즉시 빠져나와야 워크플로우가 종료 가능.
        expected_ports = set(self.dxl_controllers.keys())
        while (self._sync_done_ports | self._sync_failed_ports) != expected_ports:
            if task_control is not None and task_control.get('stop', False):
                print("[SYSTEM] Sync interrupted by stop signal.")
                return False
            time.sleep(0.05)

        if self._sync_failed_ports:
            print(f"[ERROR] Sync failed on ports: {self._sync_failed_ports}")
            return False

        print("[SUCCESS] Leader home reached. Move gripper to release torque (open then close if currently closed).")
        return True

    def get_gripper_pos(self, joint):
        gripper_pos_low = joint['joint_lower_bound']
        gripper_pos_high = joint['joint_upper_bound']
        sign = joint['sign']
        if sign < 0:
            gripper_pos = gripper_pos_high - (joint['dxl_position'] - joint['gripper_dxl_range'][1]) / (joint['gripper_dxl_range'][0] - joint['gripper_dxl_range'][1]) * (gripper_pos_high - gripper_pos_low)
        else:
            gripper_pos = (joint['dxl_position'] - joint['gripper_dxl_range'][1]) / (joint['gripper_dxl_range'][0] - joint['gripper_dxl_range'][1]) * (gripper_pos_high - gripper_pos_low) + gripper_pos_low
        if gripper_pos < gripper_pos_low:
            return gripper_pos_low
        elif gripper_pos > gripper_pos_high:
            return gripper_pos_high
        return gripper_pos

        

    def _read_dxl_loop(self, task_control):
        """
        Dynamixel 값을 최대한 빠르게 지속적으로 읽어 joint_map에 반영하는 루프.
        별도 스레드에서 실행되며, publish 루프와 분리되어 있다.
        """
        while not task_control.get('stop', False) and not task_control.get('episode_stop', False):
            try:
                self.read_dxl_and_write_to_joint_map()
                if not self._first_read_done.is_set():
                    self._first_read_done.set()
            except Exception as e:
                print(f"[ERROR] Dynamixel Read Failed: {e}", flush=True)
                task_control['stop'] = True
                self._first_read_done.set()
                break


    def position_pub(self, task_control):
        self._first_read_done = threading.Event()
        read_thread = threading.Thread(
            target=self._read_dxl_loop,
            args=(task_control,),
            daemon=True,
        )
        read_thread.start()

        if not self._first_read_done.wait(timeout=5.0):
            print("[ERROR] Dynamixel first read timeout", flush=True)
            task_control['stop'] = True
            read_thread.join(timeout=1.0)
            return

        try:
            rate = _SimpleRate(self.publish_rate)
            while not task_control.get('stop', False) and not task_control.get('episode_stop', False):

                group_by_agent = self.group_joints_by_agent()

                gripper_rad_pos = None
                for agent in self.agents:
                    joint_list = group_by_agent[agent.id]
                    for joint in joint_list:
                        if 'is_dummy_gripper' in joint and joint['is_dummy_gripper']:
                            continue
                        joint_name = joint['joint_name']
                        joint_index = agent.joint_names.index(joint_name)
                        dxl_id = joint['dxl_id']
                        position = joint['dxl_position']
                        if joint.get('is_gripper', False):
                            joint['target_agent_position'] = self.get_gripper_pos(joint)
                        else:
                            target_pos = self.get_rad_pos(joint)

                            if 'prev_agent_position' not in joint:  # 첫 번째 루프에서 이전 위치 초기화
                                joint['prev_agent_position'] = target_pos

                            # EMA 필터 적용
                            filtered = self.ema * joint['prev_agent_position'] + (1 - self.ema) * target_pos
                            joint['prev_agent_position'] = filtered

                            # Step clipping: 직전 명령값 기준으로 max_step_rad를 넘지 못하도록 제한.
                            # 첫 루프에서는 현재 관절 위치를 기준으로 초기화해 시작 스파이크를 방지.
                            if 'prev_commanded_position' not in joint:
                                current_agent_pos = joint.get('agent_position', filtered)
                                joint['prev_commanded_position'] = current_agent_pos

                            delta = filtered - joint['prev_commanded_position']
                            if delta > self.max_step_rad:
                                delta = self.max_step_rad
                            elif delta < -self.max_step_rad:
                                delta = -self.max_step_rad
                            clipped = joint['prev_commanded_position'] + delta
                            joint['prev_commanded_position'] = clipped
                            joint['target_agent_position'] = clipped


                    action = agent.fetch_joint_map_to_action(joint_list)
                    self.thread_pool.submit(agent.move_joint_step, action)

                    end = time.time()
                # self.target_pos[-1] = self.get_gripper_pos()
                #-----------------------------------------



                # 초기 토크 해제: 시작 시점에 그리퍼가 이미 닫혀 있으면 사용자가
                # 의식적으로 한 번 열었다가 다시 닫는 동작을 해야 토크가 풀린다.
                # (이미 열려 있으면 닫기만 해도 풀림 — 기존 동작과 동일)
                # (joint_map의 dxl_position은 _read_dxl_loop에서 갱신되므로 포트 동시 접근 없음)
                if self._torque_release_pending_ports:
                    for port in list(self._torque_release_pending_ports):
                        dxl_controller = self.dxl_controllers[port]
                        if not dxl_controller.gripper_dxl_ids:
                            # 그리퍼가 없는 포트는 즉시 해제
                            dxl_controller.remove_torque()
                            self._torque_release_pending_ports.discard(port)
                            print(f"[NOTICE] No gripper on {port}; torque released.", flush=True)
                            continue

                        # 모든 그리퍼의 닫힘/열림 상태를 한 번에 확인
                        all_closed = True
                        any_open = False
                        all_read = True
                        for gripper_dxl_id in dxl_controller.gripper_dxl_ids:
                            gj = self.get_joint_by_dxl_id(port, gripper_dxl_id)
                            if gj is None or 'dxl_position' not in gj:
                                all_read = False
                                break
                            gpos = gj['dxl_position']
                            grange = gj['gripper_dxl_range']
                            # 캘리브레이션 오차를 감안해 50% 지점을 threshold로 사용.
                            mid = (grange[0] + grange[1]) / 2.0
                            if grange[0] < grange[1]:
                                is_closed = gpos >= mid
                            else:
                                is_closed = gpos <= mid
                            if not is_closed:
                                all_closed = False
                                any_open = True

                        if not all_read:
                            continue

                        # 사용자가 그리퍼를 한 번이라도 연 것을 관측하면 기록
                        if any_open and not self._gripper_open_observed[port]:
                            self._gripper_open_observed[port] = True
                            print(
                                f"[NOTICE] Leader gripper opened on port {port}. "
                                f"Close it again to take control.",
                                flush=True,
                            )

                        if all_closed and not self._gripper_open_observed[port]:
                            # 시작 시점에 이미 닫혀 있는 케이스 — 토크 해제하지 않고 사용자 동작 대기
                            if not self._gripper_initial_close_logged[port]:
                                self._gripper_initial_close_logged[port] = True
                                print(
                                    f"[NOTICE] Leader gripper on port {port} is currently closed. "
                                    f"Open it once and close again to take control.",
                                    flush=True,
                                )
                            continue

                        if all_closed and self._gripper_open_observed[port]:
                            # 열었다가 닫는 사이클 완료 → 토크 해제
                            dxl_controller.remove_torque()
                            self._torque_release_pending_ports.discard(port)
                            print(f"[NOTICE] Leader torque released on port {port}.", flush=True)

                    if not self._torque_release_pending_ports:
                        self.is_synced = True
                        print("[NOTICE] All leader grippers ready. Recording can start.", flush=True)

                # 그리퍼를 벌리면 정지하도록 하는 코드--------------
                should_pause = [False] * len(self.dxl_controllers)
                should_resume = [False] * len(self.dxl_controllers)
                dxl_contoller_index = 0
                for port, dxl_controller in self.dxl_controllers.items():
                    gripper_dxl_ids = dxl_controller.gripper_dxl_ids
                    for dxl_id in gripper_dxl_ids:
                        gripper_joint = self.get_joint_by_dxl_id(port, dxl_id)
                        dxl_pos = gripper_joint['dxl_position']
                        gripper_range = gripper_joint['gripper_dxl_range']

                        if gripper_range[0] < gripper_range[1]:
                            should_pause[dxl_contoller_index] = dxl_pos < gripper_range[0] - 200
                            should_resume[dxl_contoller_index] = dxl_pos > gripper_range[0] - 100
                        else:
                            should_pause[dxl_contoller_index] = dxl_pos > gripper_range[0] + 200
                            should_resume[dxl_contoller_index] = dxl_pos < gripper_range[0] + 100

                    if not self.is_paused[dxl_contoller_index] and should_pause[dxl_contoller_index]:
                        self.is_paused[dxl_contoller_index] = True
                        dxl_controller.enable_torque()
                        time.sleep(0.1)  # 토크가 걸릴 시간을 약간 줌
                        print("Teleoperation Paused")

                    elif self.is_paused[dxl_contoller_index] and should_resume[dxl_contoller_index]:
                        self.is_paused[dxl_contoller_index] = False
                        dxl_controller.remove_torque()
                        time.sleep(0.1)
                        print("Teleoperation Resumed")
                        
                    dxl_contoller_index += 1


                rate.sleep()

        except Exception as e:
            # 예상치 못한 전체 루프 에러 처리
            import traceback
            error_msg = traceback.format_exc()
            print(f"[CRITICAL ERROR] position_pub loop crashed:\n{error_msg}", flush=True)
            # SocketIOStream을 통해 프론트엔드로도 전송됨
            raise e
        
        finally:
            # 에러가 나든 정상 종료되든 반드시 실행되는 블록.
            # 단, episode 단위로 정상 종료 (episode_stop=True) 된 경우엔 collection
            # 전체를 멈추면 안 된다 — record_episode 의 outer loop 가 다음 epoch 로
            # 진행해야 함. task_control['stop']=True 를 무조건 set 하면 다음 outer
            # iteration 진입 시 break 되어 첫 epoch 후 record 가 끝나버린다.
            # 예외 발생 또는 외부 stop 신호로 들어온 경우만 stop 을 set한다.
            print("Cleaning up Leader Robot resources...", flush=True)
            normal_episode_exit = (
                task_control.get('episode_stop', False)
                and not task_control.get('stop', False)
            )
            if not normal_episode_exit:
                task_control['stop'] = True
            try:
                read_thread.join(timeout=1.0)
                print("Dxl read thread joined.", flush=True)
            except Exception:
                print("Failed to join dxl read thread.", flush=True)
            try:
                self.thread_pool.shutdown(wait=False)
                print("ThreadPoolExecutor shut down.", flush=True)
            except:
                print("Failed to shut down ThreadPoolExecutor.", flush=True)
            for port, dxl_controller in self.dxl_controllers.items():
                try:
                    # 포트 닫기
                    dxl_controller.portHandler.closePort()
                    print(f"Port {port} closed safely.", flush=True)
                except:
                    print(f"Failed to close port {port} cleanly.", flush=True)
            self.is_cleaned_up = True


    def leader_teleop_workflow(self, task_control):
        """
        이 함수는 ProcessManager.start_function에 의해 백그라운드에서 실행됩니다.
        """
        print("[SYSTEM] Starting Leader Sync Process...")

        # 1. 동기화 실행 — task_control을 함께 넘겨서 내부 대기 루프가 stop 신호를
        #    체크하도록 한다 (sync_dxl_controller가 dxl read 에러로 죽으면 메인이
        #    무한 대기하던 버그 방지).
        sync_ok = self.sync_leader_robot(task_control=task_control)

        # 사용자 중지 또는 sync 실패 시 워크플로우 종료
        if task_control['stop']:
            print("[SYSTEM] Workflow stopped by user during sync.")
            return
        if not sync_ok:
            print("[SYSTEM] Sync failed — exiting workflow.")
            return

        print("[SUCCESS] Sync completed. Transitioning to Teleoperation...")

        # 2. 곧바로 텔레옵(무한 루프) 실행
        # leader.position_pub 내부에 task_control을 체크하는 로직이 있으면 더욱 좋습니다.
        self.position_pub(task_control=task_control)
