from gym.envs.registration import register

# roboschool envs
## pendula
register(
	id='InvertedPendulumPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.pendulum.inverted_pendulum_env:InvertedPendulumBulletEnv',
	max_episode_steps=1000,
	reward_threshold=950.0,
	)

register(
	id='InvertedDoublePendulumPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.pendulum.inverted_double_pendulum_env:InvertedDoublePendulumBulletEnv',
	max_episode_steps=1000,
	reward_threshold=9100.0,
	)

register(
	id='InvertedPendulumSwingupPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.pendulum.inverted_pendulum_env:InvertedPendulumSwingupBulletEnv',
	max_episode_steps=1000,
	reward_threshold=800.0,
	)


## manipulators
register(
	id='ReacherPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.reacher_env:ReacherBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

register(
	id='PusherPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.pusher_env:PusherBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
)

register(
	id='ThrowerPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.thrower_env:ThrowerBulletEnv',
	max_episode_steps=100,
	reward_threshold=18.0,
)

register(
	id='StrikerPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.striker_env:StrikerBulletEnv',
	max_episode_steps=100,
	reward_threshold=18.0,
)

## locomotors
register(
	id='Walker2DPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.walker2d_env:Walker2DBulletEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)
register(
	id='HalfCheetahPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.half_cheetah_env:HalfCheetahBulletEnv',
	max_episode_steps=1000,
	reward_threshold=3000.0
	)

register(
	id='AntPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.ant_env:AntBulletEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HopperPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.hopper_env:HopperBulletEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HumanoidPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.humanoid_env:HumanoidBulletEnv',
	max_episode_steps=1000
	)

register(
	id='HumanoidFlagrunPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.humanoid_flagrun_env:HumanoidFlagrunBulletEnv',
	max_episode_steps=1000,
	reward_threshold=2000.0
	)

register(
	id='HumanoidFlagrunHarderPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.humanoid_flagrun_env:HumanoidFlagrunHarderBulletEnv',
	max_episode_steps=1000
	)

register(
	id='AtlasPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.atlas_env:AtlasBulletEnv',
	max_episode_steps=1000
	)

# mujoco envs
register(
	id='InvertedPendulumMuJoCoEnv-v0',
	entry_point='pybulletgym.envs.mujoco.envs.pendulum.inverted_pendulum_env:InvertedPendulumMuJoCoEnv',
	max_episode_steps=1000,
	reward_threshold=950.0,
)

register(
	id='InvertedDoublePendulumMuJoCoEnv-v0',
	entry_point='pybulletgym.envs.mujoco.envs.pendulum.inverted_double_pendulum_env:InvertedDoublePendulumMuJoCoEnv',
	max_episode_steps=1000,
	reward_threshold=9100.0,
)

register(
	id='Walker2DMuJoCoEnv-v0',
	entry_point='pybulletgym.envs.mujoco.envs.locomotion.walker2d_env:Walker2DMuJoCoEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
)
register(
	id='HalfCheetahMuJoCoEnv-v0',
	entry_point='pybulletgym.envs.mujoco.envs.locomotion.half_cheetah_env:HalfCheetahMuJoCoEnv',
	max_episode_steps=1000,
	reward_threshold=3000.0
)

register(
	id='AntMuJoCoEnv-v0',
	entry_point='pybulletgym.envs.mujoco.envs.locomotion.ant_env:AntMuJoCoEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
)

register(
	id='HopperMuJoCoEnv-v0',
	entry_point='pybulletgym.envs.mujoco.envs.locomotion.hopper_env:HopperMuJoCoEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
)

register(
	id='HumanoidMuJoCoEnv-v0',
	entry_point='pybulletgym.envs.mujoco.envs.locomotion.humanoid_env:HumanoidMuJoCoEnv',
	max_episode_steps=1000
)


## phase-based roboschool locomotors
register(
	id='Walker2DPyBulletPhaseEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:Walker2DBulletPhaseEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)
register(
	id='HalfCheetahPyBulletPhaseEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:HalfCheetahBulletPhaseEnv',
	max_episode_steps=1000,
	reward_threshold=3000.0
	)

register(
	id='AntPyBulletPhaseEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:AntBulletPhaseEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HopperPyBulletPhaseEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:HopperBulletPhaseEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HumanoidPyBulletPhaseEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:HumanoidBulletPhaseEnv',
	max_episode_steps=1000
	)

## phase-based 'harder' roboschool locomotors - blocks thrown
register(
	id='Walker2DPyBulletPhaseHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:Walker2DBulletPhaseHarderEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)
register(
	id='HalfCheetahPyBulletPhaseHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:HalfCheetahBulletPhaseHarderEnv',
	max_episode_steps=1000,
	reward_threshold=3000.0
	)

register(
	id='AntPyBulletPhaseHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:AntBulletPhaseHarderEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HopperPyBulletPhaseHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:HopperBulletPhaseHarderEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HumanoidPyBulletPhaseHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.phase_based_envs:HumanoidBulletPhaseHarderEnv',
	max_episode_steps=1000
	)

## standard (non-phase) 'harder' roboschool locomotors - blocks thrown
register(
	id='Walker2DPyBulletHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.harder_envs:Walker2DBulletHarderEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)
register(
	id='HalfCheetahPyBulletHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.harder_envs:HalfCheetahBulletHarderEnv',
	max_episode_steps=1000,
	reward_threshold=3000.0
	)

register(
	id='AntPyBulletHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.harder_envs:AntBulletHarderEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HopperPyBulletHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.harder_envs:HopperBulletHarderEnv',
	max_episode_steps=1000,
	reward_threshold=2500.0
	)

register(
	id='HumanoidPyBulletHarderEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.locomotion.harder_envs:HumanoidBulletHarderEnv',
	max_episode_steps=1000
	)

## 3 link reacher
register(
	id='Reacher3PyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.reacher3_env:Reacher3BulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

# 2-link reacher with episode termination
register(
	id='ReacherTerminatePyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.reacher_terminate_env:ReacherTerminateBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

# 2-link reacher with episode termination and 4 goals
register(
	id='ReacherSequentialPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.reacher_sequential_env:ReacherSequentialBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

# 2-link reacher with episode termination and 4 coloured targets, random positions
register(
	id='ReacherColoursPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.reacher_colours_env:ReacherColoursBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)

# Constrained environments where reacher tip is init in same quadrant as goal:
# 2-link reacher with episode termination and 4 goals
register(
	id='ReacherSequentialConstrainedPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.reacher_sequential_constrained_env:ReacherSequentialConstrainedBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)
## 3 link reacher
register(
	id='Reacher3ConstrainedPyBulletEnv-v0',
	entry_point='pybulletgym.envs.roboschool.envs.manipulation.reacher3_constrained_env:Reacher3ConstrainedBulletEnv',
	max_episode_steps=150,
	reward_threshold=18.0,
	)


def get_list():
	envs = ['- ' + spec.id for spec in gym.pgym.envs.registry.all() if spec.id.find('Bullet') >= 0 or spec.id.find('MuJoCo') >= 0]
	return envs
