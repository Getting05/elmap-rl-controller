from typing import Union

from params_proto import Meta

from robodog_gym.envs.base.legged_robot_config import Cfg
from robodog_gym.envs.robodog.mybot_v2_1_config import config_mybot_v2_1


def config_mybot_v3(Cnfg: Union[Cfg, Meta]):
    config_mybot_v2_1(Cnfg)

    _ = Cnfg.asset
    _.file = '{MINI_GYM_ROOT_DIR}/resources/robots/mybot_v3/urdf/mybot_v3.urdf'

    _ = Cnfg.env
    _.env_spacing = 8.

    _ = Cnfg.terrain
    _.mesh_type = 'plane'
    _.measure_heights = False
    _.curriculum = False
    _.teleport_robots = False
    _.custom_meshes = []
    _.custom_trenches = {
        "depth": 2.0,
        "x_half_length": 3.95,
        "y_half_length": 3.925,
        "patterns": [
            {
                "orientation": "x",
                "trench_width": 0.1,
                "solid_width": 0.2,
            },
            {
                "orientation": "y",
                "trench_width": 0.15,
                "solid_width": 0.4,
            },
        ],
    }
