'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''

from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle

ROBOT_POSE_CLF = 'robot_pose.pkl'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF, 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        target_joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 
                         'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        postures = ['Left', 'Frog', 'StandInit', 'Right', 'Back', 'Crouch', 
                    'Belly', 'Stand', 'HeadBack', 'Knee', 'Sit']
        
        posture = 'unknown'
        # YOUR CODE HERE
        all_data = []
        for jointname in target_joints:
            all_data.append(perception.joint[jointname])
        
        # set AngleX and AngleY
        all_data.append(perception.imu[0])
        all_data.append(perception.imu[1])

        prediction = self.posture_classifier.predict([all_data])

        posture = postures[prediction[0]]

        return posture 

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
