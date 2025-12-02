#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-12-01
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEXARM_URDF_PATH_DICT

# Common config
YOCO = {
    "use_sim": True,
    "cam_type": "empty",
    "srv_port": {
        "mujoco_port": 12345,
        "robot_port": 12346,
        "camera_port": 12347,
    },
}
MIT_CFG = {
    "kp": [200.0, 200.0, 250.0, 150.0, 20.0, 20.0, 20.0],
    "kd": [5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
}
HEXARM_CFG = {"arm_type": "archer_y6", "gripper_type": "gp100_p050"}
if HEXARM_CFG["gripper_type"] == "empty":
    HEXARM_CFG["use_gripper"] = False
elif HEXARM_CFG["gripper_type"] == "gp100_p050":
    HEXARM_CFG["use_gripper"] = True

# launch params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ROBO_YOCO_DIR = f"{SCRIPT_DIR}/../../../hex_robo_yoco"
LAUNCH_PATH_DICT = {
    "driver":
    (f"{HEX_ROBO_YOCO_DIR}/../examples/basic/archer_y6/driver.py", YOCO),
}
LAUNCH_PARAMS_DICT = {"driver": {}}

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ROBO_YOCO_DIR = f"{SCRIPT_DIR}/../../../hex_robo_yoco"
NODE_PARAMS_DICT = {
    # cli
    "archer_y6_cli": {
        "name": "archer_y6_cli",
        "node_path": f"{HEX_ROBO_YOCO_DIR}/../examples/basic/archer_y6/cli.py",
        "cfg_path":
        f"{HEX_ROBO_YOCO_DIR}/../examples/basic/archer_y6/cli.json",
        "cfg": {
            "yoco":
            YOCO,
            "model_path":
            HEXARM_URDF_PATH_DICT[
                f'{HEXARM_CFG["arm_type"]}_{HEXARM_CFG["gripper_type"]}'],
            "use_gripper":
            HEXARM_CFG["use_gripper"],
            "mit_cfg":
            MIT_CFG,
            "net": {
                "mujoco_net": {
                    "port": YOCO["srv_port"]["mujoco_port"]
                },
                "robot_net": {
                    "port": YOCO["srv_port"]["robot_port"]
                },
                "camera_net": {
                    "port": YOCO["srv_port"]["camera_port"]
                },
            },
        },
    },
}


def get_node_cfgs(node_params_dict: dict = NODE_PARAMS_DICT,
                  launch_args: dict | None = None):
    launch_node_cfg = HexNodeConfig.get_launch_params_cfgs(
        launch_params_dict=LAUNCH_PARAMS_DICT,
        launch_default_params_dict=LAUNCH_PARAMS_DICT,
        launch_path_dict=LAUNCH_PATH_DICT,
    )
    node_default_params_config = HexNodeConfig(node_params_dict)
    node_default_params_config.add_cfgs(launch_node_cfg)
    node_default_params_dict = node_default_params_config.get_cfgs(
        use_list=False)
    return HexNodeConfig.parse_node_params_dict(
        node_params_dict,
        node_default_params_dict,
    )


def main():
    node_cfgs = get_node_cfgs()
    launch = HexLaunch(node_cfgs)
    launch.run()


if __name__ == '__main__':
    main()
