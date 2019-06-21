from pybulletgym.envs.roboschool.envs.locomotion.phase_walker_base_env import WalkerPhaseBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import Walker2DPhase, HopperPhase, HalfCheetahPhase, HumanoidPhase, AntPhase


class HopperBulletPhaseEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = HopperPhase()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)

class HalfCheetahBulletPhaseEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = HalfCheetahPhase()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)

class HumanoidBulletPhaseEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self, robot=HumanoidPhase()):
        self.robot = robot
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)
        self.electricity_cost = 4.25 * WalkerPhaseBaseBulletEnv.electricity_cost
        self.stall_torque_cost = 4.25 * WalkerPhaseBaseBulletEnv.stall_torque_cost

class Walker2DBulletPhaseEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = Walker2DPhase()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)

class AntBulletPhaseEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = AntPhase()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)