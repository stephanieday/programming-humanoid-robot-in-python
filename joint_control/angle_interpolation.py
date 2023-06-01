'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        # target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {} # list of joint names
        names, times, keys = keyframes

        curr_t = perception.time - self.start_time

        # Calculate bezier for each joint
        for idx1, joint_name in enumerate(names):
            for idx2, time in enumerate(times[idx1]):
                key = keys[idx1]
                if idx2 == len(times[idx1])-1:
                    break
                i = (curr_t - time) / (times[idx1][idx2 + 1] - time)
                
                if curr_t < times[idx1][0]:
                    p0 = key[0][0]
                    p1 = key[0][1][2]
                    p3 = key[1][0]
                    p2 = key[0][2][2]
                    target_joints[joint_name] = self.cubic_bezier(p0, p1, p2, p3, i)
                elif time <= curr_t < times[idx1][idx2 + 1]:
                    # parameters for all other curves
                    p0 = key[idx2][0]
                    p1 = key[idx2][1][2] + p0
                    p3 = key[idx2 + 1][0]
                    p2 = key[idx2 + 1][1][2] + p3
                    target_joints[joint_name] = self.cubic_bezier(p0, p1, p2, p3, i)
                

        if 'LHipYawPitch' in target_joints.keys():
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        return target_joints
    
    def cubic_bezier(self, p0, p1, p2, p3, i):
        """
            Compute the cubic Bezier interpolation at parameter i.
            p0, p1, p2, p3 are the control points.
        """
        # Calculate blending functions
        b0 = (1 - i) ** 3
        b1 = 3 * ((1 - i) ** 2) * i
        b2 = 3 * (1 - i) * (i ** 2)
        b3 = i ** 3

        # Compute interpolated bezier at time i
        bezier = b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3
        return bezier


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
    
