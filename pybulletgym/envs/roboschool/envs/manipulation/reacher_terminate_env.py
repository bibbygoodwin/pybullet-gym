from pybulletgym.envs.roboschool.envs.env_bases import BaseBulletEnv
from pybulletgym.envs.roboschool.scenes.scene_bases import SingleRobotEmptyScene
import numpy as np
from pybulletgym.envs.roboschool.robots.manipulators.reacher_terminate import ReacherTerminate


class ReacherTerminateBulletEnv(BaseBulletEnv):
    def __init__(self):
        self.robot = ReacherTerminate()
        BaseBulletEnv.__init__(self, self.robot)

    def create_single_player_scene(self, bullet_client):
        return SingleRobotEmptyScene(bullet_client, gravity=0.0, timestep=0.0165, frame_skip=1)

    def step(self, a):
        assert (not self.scene.multiplayer)
        self.robot.apply_action(a)
        self.scene.global_step()

        state = self.robot.calc_state()  # sets self.to_target_vec

        self.rewards = self.get_reward(a)

        if self.potential/(-100) < 0.02:
            self.done = True # this is set to False in the reset() of the base env

        self.HUD(state, a, False)
        return state, sum(self.rewards), self.done, {}

    def camera_adjust(self):
        x, y, z = self.robot.fingertip.pose().xyz()
        x *= 0.5
        y *= 0.5
        self.camera.move_and_look_at(0.3, 0.3, 0.3, x, y, z)

    def get_reward(self, a):
        potential_old = self.potential
        self.potential = self.robot.calc_potential()

        electricity_cost = (
                -0.10 * (np.abs(a[0] * self.robot.theta_dot) + np.abs(a[1] * self.robot.gamma_dot))  # work torque*angular_velocity
                - 0.01 * (np.abs(a[0]) + np.abs(a[1]))  # stall torque require some energy
        )
        stuck_joint_cost = -0.1 if np.abs(np.abs(self.robot.gamma) - 1) < 0.1 else 0.0 # was '<0.01' in roboschool

        return [float(self.potential - potential_old), float(electricity_cost), float(stuck_joint_cost)]

class ReacherTerminateL2BulletEnv(ReacherTerminateBulletEnv):
    def __init__(self):
        super().__init__()

    def get_reward(self, a):
        potential_old = self.potential
        self.potential = self.robot.calc_potential()

        electricity_cost = (
                -0.10 * (np.abs(a[0] * self.robot.theta_dot) + np.abs(a[1] * self.robot.gamma_dot))  # work torque*angular_velocity
                - 0.01 * (np.abs(a[0]) + np.abs(a[1]))  # stall torque require some energy
        )
        stuck_joint_cost = -0.1 if np.abs(np.abs(self.robot.gamma) - 1) < 0.1 else 0.0 # was '<0.01' in roboschool

        return [float(self.potential - potential_old), float(electricity_cost), float(stuck_joint_cost)]
