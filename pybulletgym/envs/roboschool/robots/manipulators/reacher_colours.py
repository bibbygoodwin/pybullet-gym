import itertools
import numpy as np

from pybulletgym.envs.roboschool.robots.robot_bases import MJCFBasedRobot
import pybullet


class ReacherColours(MJCFBasedRobot):
    DEFAULT_GOAL_ORDERS = np.array(sorted(itertools.permutations(range(4), 4)))
    TARG_LIMIT = 0.116 # gives approx 10° angles in each link
    target_names = ["target_1", "target_2", "target_3", "target_4"]
    target_rgba = [[1, 1, 0.2, 1], [0.2, 1, 0.6, 1], [0.2, 0.2, 1, 1], [1, 0.2, 0.6, 1]]

    def __init__(self):
        MJCFBasedRobot.__init__(self, 'reacher_4goal.xml', 'body0', action_dim=2, obs_dim=9)
        self.goal_positions = np.array(
            [
                [self.TARG_LIMIT, self.TARG_LIMIT],
                [self.TARG_LIMIT, -self.TARG_LIMIT],
                [-self.TARG_LIMIT, -self.TARG_LIMIT],
                [-self.TARG_LIMIT, self.TARG_LIMIT]
            ],  # defined clockwise NE->SE->SW->NW
            dtype=np.float32
        )

    def robot_specific_reset(self, bullet_client):
        self.goal = self.goal_positions[self.np_random.randint(4)]
        # self.jdict["target_x_1"].reset_current_position(self.goal[0], 0)
        # self.jdict["target_y_1"].reset_current_position(self.goal[1], 0)
        #
        # other_targets = ["target_x_2", "target_y_2",
        #                  "target_x_3", "target_y_3",
        #                  "target_x_4", "target_y_4"]
        # for t in other_targets:
        #     self.jdict[t].reset_current_position(self.TARG_LIMIT*np.random.uniform(), 0)

        self.fingertip = self.parts["fingertip"]
        self.target = self.parts["target_1"]
        self.central_joint = self.jdict["joint0"]
        self.elbow_joint = self.jdict["joint1"]
        self.central_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.elbow_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        for target, rgba in zip(self.target_names, self.target_rgba):
            pybullet.changeVisualShape(self.parts[target].bodyIndex, 2, rgbaColor=rgba)


    def apply_action(self, a):
        assert (np.isfinite(a).all())
        self.central_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
        self.elbow_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))

    def calc_state(self):
        theta, self.theta_dot = self.central_joint.current_relative_position()
        self.gamma, self.gamma_dot = self.elbow_joint.current_relative_position()
        target_x, _ = self.jdict["target_x_1"].current_position()
        target_y, _ = self.jdict["target_y_1"].current_position()
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
