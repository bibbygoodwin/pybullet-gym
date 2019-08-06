from pybulletgym.envs.roboschool.envs.locomotion.phase_walker_base_env import WalkerPhaseBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import Walker2DPhase, HopperPhase, HalfCheetahPhase, HumanoidPhase, AntPhase
from pybulletgym.envs.roboschool.robots.locomotors import Walker2DPhaseHarder, HopperPhaseHarder, HalfCheetahPhaseHarder, HumanoidPhaseHarder, AntPhaseHarder


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

"""
'Harder' versions of the phase-based envs, meaning blocks are chucked at the robots
"""
class HopperBulletPhaseHarderEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = HopperPhaseHarder()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)

class HalfCheetahBulletPhaseHarderEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = HalfCheetahPhaseHarder()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)

class HumanoidBulletPhaseHarderEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self, robot=HumanoidPhaseHarder()):
        self.robot = robot
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)
        self.electricity_cost = 4.25 * WalkerPhaseBaseBulletEnv.electricity_cost
        self.stall_torque_cost = 4.25 * WalkerPhaseBaseBulletEnv.stall_torque_cost

class Walker2DBulletPhaseHarderEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = Walker2DPhaseHarder()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)

class AntBulletPhaseHarderEnv(WalkerPhaseBaseBulletEnv):
    def __init__(self):
        self.robot = AntPhaseHarder()
        WalkerPhaseBaseBulletEnv.__init__(self, self.robot)