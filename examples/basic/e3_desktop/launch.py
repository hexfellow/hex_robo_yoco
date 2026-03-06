#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEXARM_URDF_PATH_DICT
from hex_robo_yoco import HEX_YOCO_DRIVER_PATH_DICT

# YOCO config
HEXARM_CFG = {"arm_type": "archer_y6", "gripper_type": "gp100_p050"}
YOCO = {
    "use_sim": True,
    "cam_type": ["empty", "empty", "empty"],
    "srv_port": {
        "mujoco_port": 12345,
        "left_robot_port": 12346,
        "right_robot_port": 12347,
        "head_camera_port": 12348,
        "left_camera_port": 12349,
        "right_camera_port": 12350,
    },
    "params": {
        "mujoco": {
            "headless": False,
        },
        "robot": {
            "mit_kp": [200.0, 200.0, 250.0, 150.0, 20.0, 20.0, 20.0],
            "mit_kd": [5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
            "arm_type": HEXARM_CFG["arm_type"],
        },
        "rgb": {
            "resolution": [640, 480],
            "crop": [0, 640, 0, 480],
            "exposure": 70,
            "temperature": 0,
        },
        "realsense": {
            "resolution": [640, 480],
        },
        "berxel": {
            "exposure": 10000,
            "gain": 100,
        },
    },
    "device": {
        "left_robot": {
            "device_ip": "172.18.30.133",
            "device_port": 8439,
        },
        "right_robot": {
            "device_ip": "172.18.30.133",
            "device_port": 9439,
        },
        "head_camera": {
            "serial_number": "243422071854",
        },
        "left_camera": {
            "serial_number": "243422073194",
        },
        "right_camera": {
            "serial_number": "243422071878",
        },
    },
}

# launch params
LAUNCH_PATH_DICT = {
    "driver": (HEX_YOCO_DRIVER_PATH_DICT["e3_desktop"], YOCO),
}
LAUNCH_PARAMS_DICT = {"driver": {}}

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = f"{SCRIPT_DIR}/../../.."
NODE_PARAMS_DICT = {
    # cli
    "e3_desktop_cli": {
        "name": "e3_desktop_cli",
        "node_path": f"{BASE_DIR}/examples/basic/e3_desktop/cli.py",
        "cfg_path": f"{BASE_DIR}/examples/basic/e3_desktop/cli.json",
        "cfg": {
            "yoco":
            YOCO,
            "model_path":
            HEXARM_URDF_PATH_DICT[
                f'{HEXARM_CFG["arm_type"]}_{HEXARM_CFG["gripper_type"]}'],
            "mit_cfg": {
                "kp": YOCO["params"]["robot"]["mit_kp"],
                "kd": YOCO["params"]["robot"]["mit_kd"],
            },
            "net": {
                "mujoco_net": {
                    "port": YOCO["srv_port"]["mujoco_port"]
                },
                "left_robot_net": {
                    "port": YOCO["srv_port"]["left_robot_port"]
                },
                "right_robot_net": {
                    "port": YOCO["srv_port"]["right_robot_port"]
                },
                "head_camera_net": {
                    "port": YOCO["srv_port"]["head_camera_port"]
                },
                "left_camera_net": {
                    "port": YOCO["srv_port"]["left_camera_port"]
                },
                "right_camera_net": {
                    "port": YOCO["srv_port"]["right_camera_port"]
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
