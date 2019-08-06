from pybulletgym.envs.roboschool.envs.locomotion.walker_base_env import WalkerBaseBulletEnv
from pybulletgym.envs.roboschool.robots.locomotors import Walker2DHarder, HopperHarder, HalfCheetahHarder, HumanoidHarder, AntHarder

"""
'Harder' versions of the non-phase, pybulletgym locomotion envs, meaning blocks are chucked at the robots
"""
class AntBulletHarderEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = AntHarder()
        WalkerBaseBulletEnv.__init__(self, self.robot)

class HalfCheetahBulletHarderEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = HalfCheetahHarder()
        WalkerBaseBulletEnv.__init__(self, self.robot)

class HopperBulletHarderEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = HopperHarder()
        WalkerBaseBulletEnv.__init__(self, self.robot)

class HumanoidBulletHarderEnv(WalkerBaseBulletEnv):
    def __init__(self, robot=HumanoidHarder()):
        self.robot = robot
        WalkerBaseBulletEnv.__init__(self, self.robot)
        self.electricity_cost = 4.25 * WalkerBaseBulletEnv.electricity_cost
        self.stall_torque_cost = 4.25 * WalkerBaseBulletEnv.stall_torque_cost

class Walker2DBulletHarderEnv(WalkerBaseBulletEnv):
    def __init__(self):
        self.robot = Walker2DHarder()
        WalkerBaseBulletEnv.__init__(self, self.robot)


