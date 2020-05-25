from pybulletgym.envs.roboschool.envs.env_bases import BaseBulletEnv
from pybulletgym.envs.roboschool.scenes.scene_bases import SingleRobotEmptyScene
import numpy as np
from pybulletgym.envs.roboschool.robots.manipulators.reacher_colours import ReacherColours

import pybullet

class ReacherColoursBulletEnv(BaseBulletEnv):
    def __init__(self):
        self.robot = ReacherColours()
        BaseBulletEnv.__init__(self, self.robot)
        self._cam_dist = 5
        self._cam_yaw = 0
        self._cam_pitch = -45
        self._render_width = 240
        self._render_height = 240
        # self._render_width = 2400
        # self._render_height = 2400

    def create_single_player_scene(self, bullet_client):
        return SingleRobotEmptyScene(bullet_client, gravity=0.0, timestep=0.0165, frame_skip=1)

    def step(self, a):
        assert (not self.scene.multiplayer)
        self.robot.apply_action(a)
        self.scene.global_step()

        state = self.robot.calc_state()  # sets self.to_target_vec

        potential_old = self.potential
        self.potential = self.robot.calc_potential()

        if self.potential/(-100) < 0.02:
            self.done = True
            self.rewards.append(5.0)

        electricity_cost = (
                -0.10 * (np.abs(a[0] * self.robot.theta_dot) + np.abs(a[1] * self.robot.gamma_dot))  # work torque*angular_velocity
                - 0.01 * (np.abs(a[0]) + np.abs(a[1]))  # stall torque require some energy
        )
        stuck_joint_cost = -0.1 if np.abs(np.abs(self.robot.gamma) - 1) < 0.01 else 0.0
        self.rewards = [float(self.potential - potential_old), float(electricity_cost), float(stuck_joint_cost)]
        self.HUD(state, a, False)
        return state, sum(self.rewards), self.done, {}

    def camera_adjust(self):
        x, y, z = self.robot.fingertip.pose().xyz()
        x *= 0.5
        y *= 0.5
        self.camera.move_and_look_at(0.3, 0.3, 0.3, x, y, z)

    def render(self, mode, close=False):
        if mode == "human":
            self.isRender = True
        if mode != "rgb_array":
            return np.array([])

        base_pos = [0,0,0]
        if hasattr(self,'robot'):
            if hasattr(self.robot,'body_xyz'):
                base_pos = self.robot.body_xyz

        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=self._cam_dist,
            yaw=self._cam_yaw,
            pitch=self._cam_pitch,
            roll=0,
            upAxisIndex=2)
        proj_matrix = self._p.computeProjectionMatrixFOV(
            fov=9, aspect=float(self._render_width)/self._render_height,
            nearVal=0.1, farVal=100.0)
        # img_arr = self._p.getCameraImage(
        #     width=self._render_width, height=self._render_height, viewMatrix=view_matrix,
        #     projectionMatrix=proj_matrix,
        #     renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
        # )
        img_arr = self._p.getCameraImage(
            width=self._render_width, height=self._render_height, viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
        )

        w = img_arr[0]  # width of the image, in pixels
        h = img_arr[1]  # height of the image, in pixels
        rgb = img_arr[2]  # color data RGB
        dep = img_arr[3]  # depth data
        print(dep.shape)
        np_img_arr = np.reshape(rgb, (h, w, 4))
        np_img_arr = np_img_arr * (1. / 255.)
        rgb_array = np_img_arr
        # rgb_array = np.array(px)
        # rgb_array = rgb_array[:, :, :3]

        return rgb_array
