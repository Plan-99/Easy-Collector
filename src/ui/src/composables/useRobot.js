import { api } from 'boot/axios';
import { useROS } from './useROS';
import { Notify } from 'quasar';

export function useRobot(robot, robotOnCallback=() => {}) {
  const { createSubscriber, createPublisher, connectROS, sendJointState } = useROS();
  let robotTopicChecker = null;

  checkRobotTopic(1);

  connectROS();

  let jointSub = null;
  let publishJointPos = () => {};

  function status() {
    return robot.status
  }

  function startRobot() {
    robot.status = 'loading'
    return api.post('/robot:start', robot).then(() => {
      checkRobotTopic()
    }).catch((error) => {
      console.error('Error starting robot:', error);
      robot.status = 'off';
    });
  }

  function stopRobot() {
    robot.status = 'loading'
    return api.post('/robot:stop', robot).then(() => {
      robot.status = 'off';
    }).catch((error) => {
      console.error('Error stopping robot:', error);
      robot.status = 'on';
    });
  }

  function subscribeRobot(callback) {
    if (robot.status === 'off') {
      return;
    }
    if (jointSub) {
      jointSub.unsubscribe();
    }
    jointSub = createSubscriber(robot.read_topic, robot.read_topic_msg, callback);
  }

  function publishRobot() {
    if (robot.status === 'off') {
      return;
    }
    publishJointPos = createPublisher(robot.write_topic, robot.write_topic_msg);
  }

  function unSubscribeRobot() {
    if (jointSub) {
      jointSub.unsubscribe();
      jointSub = null;
    }
  }

  function moveRobot(joint_index, joint_pos) {
    if (!robot) return;
    robot.joint_pos[joint_index] = joint_pos;
    sendJointState(robot.joint_names, robot.joint_pos, publishJointPos);
  }

  function goOriginPos() {
    if (!robot) return;
    const robot = robot;
    api.post(`/robot/${robot.id}/:move_to`, {
      goal_pos: [0, 0, 0, 0, 0, 0]
    }).then(() => {
      Notify.create({
        color: 'positive',
        message: 'Robot moved to origin position'
      });
    }).catch((error) => {
      console.error('Error moving robot to origin:', error);
      Notify.create({
        color: 'negative',
        message: 'Failed to move robot to origin'
      });
    });
  }

  function checkRobotTopic(maxSteps = 10) {
    if (robotTopicChecker) {
      clearInterval(robotTopicChecker);
    }
    let steps = 0;
    robotTopicChecker = setInterval(() => {
      api.get(`/topics`)
      .then((res) => {
        const isPublished = Boolean(res.data.topics.find(topic => topic.name === robot.read_topic));
        if (isPublished) {
          robot.status = 'on';
          robotOnCallback();
          clearInterval(robotTopicChecker);
        }
      })
      .catch(error => {
        console.error('Error setting up WebRTC:', error);
      });
      steps++;
      if (steps >= maxSteps) {
        clearInterval(robotTopicChecker);
        robot.status = 'off';
      }
    }, 1000);
  }

  return {
    startRobot,
    stopRobot,
    status,
    moveRobot,
    goOriginPos,
    subscribeRobot,
    publishRobot,
    unSubscribeRobot,
  };
}
