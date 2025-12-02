#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-12-01
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT
from hex_zmq_servers import HEXARM_URDF_PATH_DICT
from importlib.util import find_spec

# Common config
_HAS_BERXEL = find_spec("berxel_py_wrapper") is not None
_HAS_REALSENSE = find_spec("pyrealsense2") is not None
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

# Mujoco srv
MUJOCO_ARCHER_Y6_SRV = {
    "name": "mujoco_archer_y6_srv",
    "node_path": HEX_ZMQ_SERVERS_PATH_DICT["mujoco_archer_y6"],
    "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["mujoco_archer_y6"],
    "cfg": {
        "net": {
            "ip": "127.0.0.1",
            "port": YOCO["srv_port"]["mujoco_port"],
        },
        "params": {
            "states_rate": 1000,
            "img_rate": 30,
            "tau_ctrl": False,
            "headless": True,
            "sens_ts": True,
            "mit_kp": MIT_CFG["kp"],
            "mit_kd": MIT_CFG["kd"],
            "cam_type": YOCO["cam_type"],
        },
    },
}

# Robot srv
HEXARM_CFG = {"arm_type": "archer_y6", "gripper_type": "gp100_p050"}
if HEXARM_CFG["gripper_type"] == "empty":
    HEXARM_CFG["use_gripper"] = False
elif HEXARM_CFG["gripper_type"] == "gp100_p050":
    HEXARM_CFG["use_gripper"] = True
ROBOT_HEXARM_SRV = {
    "name": "robot_archer_y6_srv",
    "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],
    "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],
    "cfg": {
        "net": {
            "port": YOCO["srv_port"]["robot_port"],
        },
        "params": {
            "device_ip": "172.18.8.161",
            "device_port": 8439,
            "control_hz": 1000,
            "sens_ts": True,
            "arm_type": HEXARM_CFG["arm_type"],
            "use_gripper": HEXARM_CFG["use_gripper"],
            "mit_kp": MIT_CFG["kp"],
            "mit_kd": MIT_CFG["kd"],
        },
    },
}

# RGB srv
RGB_SRV = {
    "name": "camera_archer_y6_srv",
    "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_rgb"],
    "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_rgb"],
    "cfg": {
        "net": {
            "port": YOCO["srv_port"]["camera_port"],
        },
        "params": {
            "cam_path": "/dev/video0",
            "resolution": [640, 480],
            "crop": [0, 640, 0, 480],
            "exposure": 70,
            "temperature": 0,
            "frame_rate": 30,
            "sens_ts": True,
        },
    },
}

# Realsense srv
if _HAS_REALSENSE:
    REALSENSE_SRV = {
        "name": "camera_archer_y6_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_realsense"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_realsense"],
        "cfg": {
            "net": {
                "port": YOCO["srv_port"]["camera_port"],
            },
            "params": {
                "serial_number": "243422073194",
                "resolution": [640, 480],
                "frame_rate": 30,
                "sens_ts": True,
            },
        },
    }

# Berxel srv
if _HAS_BERXEL:
    BERXEL_SRV = {
        "name": "camera_archer_y6_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_berxel"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_berxel"],
        "cfg": {
            "net": {
                "port": YOCO["srv_port"]["camera_port"],
            },
            "params": {
                "serial_number": "P100RYB4C03M2B322",
                "exposure": 10000,
                "gain": 100,
                "frame_rate": 30,
                "sens_ts": True,
            },
        },
    }

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
                  launch_args: dict = YOCO):
    default_node_params_dict = NODE_PARAMS_DICT.copy()
    use_sim = launch_args.get("use_sim", YOCO["use_sim"])
    cam_type = launch_args.get("cam_type", YOCO["cam_type"])
    srv_port = launch_args.get("srv_port", YOCO["srv_port"])
    if use_sim:
        default_node_params_dict["mujoco_archer_y6_srv"] = MUJOCO_ARCHER_Y6_SRV
        default_node_params_dict["mujoco_archer_y6_srv"]["cfg"]["params"][
            "cam_type"] = cam_type
        default_node_params_dict["mujoco_archer_y6_srv"]["cfg"]["net"][
            "port"] = srv_port.get("mujoco_port",
                                   YOCO["srv_port"]["mujoco_port"])
    else:
        default_node_params_dict["robot_archer_y6_srv"] = ROBOT_HEXARM_SRV
        # cam_type: empty, rgb, realsense, berxel
        if cam_type == "rgb":
            default_node_params_dict["camera_archer_y6_srv"] = RGB_SRV
        elif cam_type == "realsense":
            if _HAS_REALSENSE:
                default_node_params_dict[
                    "camera_archer_y6_srv"] = REALSENSE_SRV
            default_node_params_dict["camera_archer_y6_srv"]["cfg"]["net"][
                "port"] = srv_port.get("camera_port",
                                       YOCO["srv_port"]["camera_port"])
        elif cam_type == "berxel":
            if _HAS_BERXEL:
                default_node_params_dict["camera_archer_y6_srv"] = BERXEL_SRV
                default_node_params_dict["camera_archer_y6_srv"]["cfg"]["net"][
                    "port"] = srv_port.get("camera_port",
                                           YOCO["srv_port"]["camera_port"])
        elif cam_type == "empty":
            pass
        else:
            raise ValueError(f"unknown camera type: {cam_type}")

    return HexNodeConfig(
        HexNodeConfig.parse_node_params_dict(
            node_params_dict,
            default_node_params_dict,
        ))


def main():
    node_cfgs = get_node_cfgs()
    launch = HexLaunch(node_cfgs)
    launch.run()


if __name__ == '__main__':
    main()
