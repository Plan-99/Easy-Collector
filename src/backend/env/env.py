import collections
import dm_env
import time
from ..env.agent import Agent
import rospy
from sensor_msgs.msg import Image, CompressedImage
from ..utils.image_parser import ros_image_to_numpy

class Env:
    def __init__(self, robots, sensors):
        self.sensors = sensors
        self.agents = []
        for robot in robots:
            self.agents.append(Agent(robot))

        for sensor in sensors:
            setattr(self, f'sensor_{sensor["id"]}', None)
            rospy.Subscriber(sensor['topic'], CompressedImage, self.image_raw_cb, callback_args=sensor['id'])

    def image_raw_cb(self, data, sensor_id):
        image = ros_image_to_numpy(data)

        setattr(self, f'sensor_{sensor_id}', image)

                
    def get_observation(self):
        obs = collections.OrderedDict()
        obs['robot_states'] = self.get_robot_states()
        obs['images'] = self.get_images()
        return obs
    
    
    def reset(self):
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())


    def record_step(self):
        time.sleep(0.1)
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
                rospy.loginfo(f"Waiting for image from sensor_{sensor['id']}")
                time.sleep(0.1)
                image = getattr(self, f"sensor_{sensor['id']}")
            image_dict[f"sensor_{sensor['id']}"] = image
        return image_dict
    

    def get_robot_states(self):
        robot_state_dict = dict()
        for agent in self.agents:
            robot_state_dict[agent.id] = {
                'qpos': agent.joint_states,
                'qaction': agent.joint_actions,
            }
        return robot_state_dict
    
    
    def get_reward(self):
        # Implement your reward logic here
        return 0.0