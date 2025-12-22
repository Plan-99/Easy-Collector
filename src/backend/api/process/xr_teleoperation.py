import numpy as np
import time
import os
import sys
from multiprocessing import shared_memory
from scipy.spatial.transform import Rotation as R

# 로깅 설정
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger = logging_mp.get_logger(__name__)

# 경로 설정
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from televuer import TeleVuerWrapper

class XRTeloperator:
    def __init__(self, left_arm_agent=None, right_arm_agent=None, left_tool_agent=None, right_tool_agent=None):
        self.left_arm_agent = left_arm_agent
        self.right_arm_agent = right_arm_agent
        self.left_tool_agent = left_tool_agent
        self.right_tool_agent = right_tool_agent
        self.frequency = 30
        
        # 이미지 및 공유 메모리 설정
        self.img_shape = (480, 640, 3)
        self.tv_img_shm = shared_memory.SharedMemory(
            create=True, 
            size=np.prod(self.img_shape) * np.uint8().itemsize
        )
        
        # TeleVuer 초기화
        self.tv_wrapper = TeleVuerWrapper(
            binocular=(self.img_shape[1] > 640),
            use_hand_tracking=True,
            img_shape=self.img_shape,
            img_shm_name=self.tv_img_shm.name,
            return_state_data=True,
        )
        
        # 이전 포즈 저장 변수
        self.prev_l_6d = None
        self.prev_r_6d = None

    @staticmethod
    def matrix_to_6d(matrix):
        """4x4 행렬에서 [x, y, z, r, p, y] 추출"""
        if matrix is None: return None
        pos = matrix[:3, 3]
        euler = R.from_matrix(matrix[:3, :3]).as_euler('xyz', degrees=False)
        return np.concatenate([pos, euler])

    def run(self, task_control, socketio_instance=None, log_emit_id=None):
        """
        메인 제어 루프
        task_control['read'] 가 True일 때만 로봇에 명령을 전송합니다.
        """
        logger.info("TeleOp Controller 루프 시작")

        print('XR is Connected. Please Open the Browser and Click [Virtual Reality] Button.')
        socketio_instance.emit(log_emit_id, {
            'log': 'XR is Connected. Please Open the Browser and Click [Virtual Reality] Button.',
            'type': 'stdout'
        }) if socketio_instance and log_emit_id else None
        
        step = 0
        try:
            while not task_control.get('stop', False):
                loop_start = time.time()
                
                # XR 기기 데이터 획득 (read 여부와 상관없이 트래킹은 유지)
                tele_data = self.tv_wrapper.get_motion_state_data()
                curr_l_6d = self.matrix_to_6d(tele_data.left_arm_pose)
                curr_r_6d = self.matrix_to_6d(tele_data.right_arm_pose)
                # task_control['read']가 True일 때만 실제 제어 수행

                if task_control.get('read', False):
                    step += 1
                    if step > 500:
                        break
                    if self.prev_l_6d is not None and self.prev_r_6d is not None:
                        # Delta 계산
                        delta_l = (curr_l_6d - self.prev_l_6d) * 10
                        delta_r = (curr_r_6d - self.prev_r_6d) * 10
                        
                        print(f"Delta L: {delta_l.round(4)}, Delta R: {delta_r.round(4)}")
                        if self.left_arm_agent.id != self.right_arm_agent.id:
                            self.left_arm_agent.move_ee_delta_step({'ee': delta_l})
                            self.right_arm_agent.move_ee_delta_step({'ee': delta_r})
                        
                        # if self.agent is not None:
                        #     self.agent.move_ee_delta_step(left_delta=delta_l, right_delta=delta_r)
                        #     logger.debug(f"Delta 전송 완료: L{delta_l.round(4)}, R{delta_r.round(4)}")
                    else:
                        logger.info("첫 프레임 포즈 캡처 완료. 다음 프레임부터 제어를 시작합니다.")
                else:
                    # 'read'가 False일 때는 이전 포즈를 업데이트하지 않거나 
                    # 현재 포즈로 계속 동기화하여 'read'가 True가 되는 순간 튐 현상을 방지합니다.
                    pass

                # 이전 포즈 업데이트 (제어 중이 아닐 때도 현재 손 위치를 추적해야 
                # 나중에 'read'가 True가 되었을 때 로봇이 갑자기 튀지 않습니다.)
                self.prev_l_6d = curr_l_6d
                self.prev_r_6d = curr_r_6d

                # 주기 제어
                elapsed = time.time() - loop_start
                time.sleep(max(0, (1.0 / self.frequency) - elapsed))


                
        except Exception as e:
            logger.error(f"Controller Loop Error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """자원 해제"""
        try:
            self.tv_img_shm.close()
            self.tv_img_shm.unlink()
            logger.info("공유 메모리 해제 및 클린업 완료")
        except:
            pass

# 사용 예시 (Conceptual)
# if __name__ == "__main__":
#     args = parse_args()
#     agent = MyRobotAgent()
#     controller = DeltaTeleOpController(args, agent)
#     task_control = {'read': False, 'stop': False}
#     
#     # 별도 스레드에서 controller.run(task_control) 실행 후
#     # 필요할 때 task_control['read'] = True 로 변경