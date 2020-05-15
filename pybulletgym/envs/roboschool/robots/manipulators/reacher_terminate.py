from pybulletgym.envs.roboschool.robots.robot_bases import MJCFBasedRobot
import numpy as np
import pybullet

class ReacherTerminate(MJCFBasedRobot):
    MAX_REACH = 0.2

    def __init__(self):
        MJCFBasedRobot.__init__(self, 'reacher.xml', 'body0', action_dim=2, obs_dim=9)

    def set_position(self, position, joint, **kwargs):
        joint._p.setJointMotorControl2(joint.bodies[joint.bodyIndex], joint.jointIndex, pybullet.POSITION_CONTROL,
                                      targetPosition=position, **kwargs)

    def robot_specific_reset(self, bullet_client):
        while True:
            self.goal = self.np_random.uniform(low=-self.MAX_REACH, high=self.MAX_REACH, size=2)
            if np.linalg.norm(self.goal) < 0.2:
                break

        self.jdict["target_x"].reset_current_position(self.goal[0], 0)
        self.jdict["target_y"].reset_current_position(self.goal[1], 0)
        self.fingertip = self.parts["fingertip"]
        self.target = self.parts["target"]
        self.central_joint = self.jdict["joint0"]
        self.elbow_joint = self.jdict["joint1"]
        self.central_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.elbow_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)

    def apply_action(self, a):
        assert (np.isfinite(a).all())
        self.central_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
        self.elbow_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))
        # self.set_position(a[0], self.central_joint, positionGain=0.1, velocityGain=1)
        # self.set_position(a[1], self.central_joint, positionGain=0.1, velocityGain=1)

    def calc_state(self):
        theta, self.theta_dot = self.central_joint.current_relative_position()
        self.gamma, self.gamma_dot = self.elbow_joint.current_relative_position()
        # print(self.gamma)
        target_x, _ = self.jdict["target_x"].current_position()
        target_y, _ = self.jdict["target_y"].current_position()
        self.to_target_vec = np.array(self.fingertip.pose().xyz()) - np.array(self.target.pose().xyz())
        return np.array([
            target_x,
            target_y,
            self.to_target_vec[0],
            self.to_target_vec[1],
            np.cos(theta),
            np.sin(theta),
            self.theta_dot,
            self.gamma,
            self.gamma_dot,
        ])

    def calc_potential(self):
        return -100 * np.linalg.norm(self.to_target_vec)
