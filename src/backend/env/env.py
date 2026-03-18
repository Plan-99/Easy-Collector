import collections
import dm_env
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from ..utils.image_parser import ros_image_to_numpy

class Env:
    def __init__(self, node, agents, sensors, language_instruction=None, virtual_agents=False):
        self.sensors = sensors
        self.node = node
        self.agents = agents
        self.language_instruction = language_instruction
        self.virtual_agents = virtual_agents

        self._sensor_subs = []
        for sensor in sensors:
            setattr(self, f'sensor_{sensor["id"]}', None)
            sub = node.create_subscription(CompressedImage, sensor['read_topic'], lambda msg, sid=sensor['id']: self.image_raw_cb(msg, sid), 10)
            self._sensor_subs.append(sub)

    def image_raw_cb(self, data, sensor_id):
        try:
            image = ros_image_to_numpy(data)
            setattr(self, f'sensor_{sensor_id}', image)
        except Exception as e:
            print(f"[ERROR] image_raw_cb (sensor {sensor_id}): {e}")

                
    def get_observation(self):
        obs = collections.OrderedDict()
        obs['robot_states'] = self.get_robot_states()
        obs['images'] = self.get_images()
        obs['language_instruction'] = self.get_language_instruction()
        return obs
    
    
    def reset(self):
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())


    def record_step(self):
        # time.sleep(0.1)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())
        

    def get_images(self):
        image_dict = dict()
        for sensor in self.sensors:
            image = getattr(self, f"sensor_{sensor['id']}")
            while image is None:
                time.sleep(0.1)
                image = getattr(self, f"sensor_{sensor['id']}")
            image_dict[f"sensor_{sensor['id']}"] = image
        return image_dict
    

    def get_robot_states(self):
        robot_state_dict = dict()
        for agent in self.agents:
            if self.virtual_agents:
                zeros = [0.0] * agent.joint_len
                qpos = zeros
                qaction = zeros
            else:
                qpos = agent.get_joint_states()
                qaction = agent.get_joint_actions()
            robot_state_dict[agent.id] = {
                'qpos': qpos,
                'qaction': qaction,
                'eepos': agent.get_ee_position(),
            }
        return robot_state_dict
    
    def get_language_instruction(self):
        return self.language_instruction
    
    
    def get_reward(self):
        # Implement your reward logic here
        return 0.0

    def destroy(self):
        """센서 구독을 해제하여 리소스 누수를 방지한다."""
        for sub in self._sensor_subs:
            self.node.destroy_subscription(sub)
        self._sensor_subs.clear()