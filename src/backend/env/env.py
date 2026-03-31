import collections
import dm_env
import time
from ..env.agent import Agent
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from ..utils.image_parser import ros_image_to_numpy

class Env:
    def __init__(self, node, agents, sensors, language_instruction=None):
        self.sensors = sensors
        self.node = node
        self.agents = agents
        self.language_instruction = language_instruction

        for sensor in sensors:
            setattr(self, f'sensor_{sensor["id"]}', None)
            msg_type = CompressedImage
            if sensor.get('read_topic_msg') == 'sensor_msgs/Image':
                msg_type = Image
            node.create_subscription(msg_type, sensor['read_topic'], lambda msg, sid=sensor['id']: self.image_raw_cb(msg, sid), 10)

    def image_raw_cb(self, data, sensor_id):
        image = ros_image_to_numpy(data)

        setattr(self, f'sensor_{sensor_id}', image)

                
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
            qpos = agent.get_joint_states()
            qaction = agent.get_joint_actions()
            qaction_delta = [qaction[i] - qpos[i] for i in range(len(qpos))]
            if agent.ik_solver is None:
                eepos = None
                eetarget = None
                eetarget_delta = None
            else:
                eepos = agent.get_ee_position()
                eetarget = agent.get_ee_target()
                eetarget_delta = {key: [eetarget[key][i] - eepos[key][i] for i in range(6)] for key in eetarget} if eetarget is not None else None
            robot_state_dict[agent.id] = {
                'qpos': agent.get_joint_states(),
                'qaction': agent.get_joint_actions(),
                'eepos': agent.get_ee_position(),
                'eetarget': agent.get_ee_target(),
                'qaction_delta': qaction_delta,
                'eetarget_delta': eetarget_delta,
            }
        return robot_state_dict
    
    def get_language_instruction(self):
        return self.language_instruction
    
    
    def get_reward(self):
        # Implement your reward logic here
        return 0.0