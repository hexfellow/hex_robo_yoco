#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT
from hex_zmq_servers import HEXARM_URDF_PATH_DICT

# Yoco config
YOCO = {"use_sim": True, "cam_type": ["berxel", "berxel", "berxel"]}

# Mit config
MIT_CFG = {
    "kp": [200.0, 200.0, 250.0, 150.0, 100.0, 100.0, 20.0],
    "kd": [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 1.0]
}

# Hexarm config
HEXARM_CFG = {"arm_type": "archer_y6", "gripper_type": "gp100_p050"}
if HEXARM_CFG["gripper_type"] == "empty":
    HEXARM_CFG["use_gripper"] = False
elif HEXARM_CFG["gripper_type"] == "gp100_p050":
    HEXARM_CFG["use_gripper"] = True

# Server config
SRV_CFG = {
    "mujoco_port": 12345,
    "left_robot_port": 12346,
    "right_robot_port": 12347,
    "head_camera_port": 12348,
    "left_camera_port": 12349,
    "right_camera_port": 12350,
}

# Mujoco config
MUJOCO_PARAMS = {
    "states_rate": 1000,
    "img_rate": 30,
    "tau_ctrl": False,
    "headless": True,
    "sens_ts": True,
    "mit_kp": MIT_CFG["kp"],
    "mit_kd": MIT_CFG["kd"],
    "cam_type": YOCO["cam_type"],
}

# Robot config
LEFT_ROBOT_PARAMS = {
    "device_ip": "172.18.8.161",
    "device_port": 8439,
    "control_hz": 1000,
    "sens_ts": True,
    "arm_type": HEXARM_CFG["arm_type"],
    "use_gripper": HEXARM_CFG["use_gripper"],
    "mit_kp": MIT_CFG["kp"],
    "mit_kd": MIT_CFG["kd"],
}
RIGHT_ROBOT_PARAMS = {
    "device_ip": "172.18.8.161",
    "device_port": 8439,
    "control_hz": 1000,
    "sens_ts": True,
    "arm_type": HEXARM_CFG["arm_type"],
    "use_gripper": HEXARM_CFG["use_gripper"],
    "mit_kp": MIT_CFG["kp"],
    "mit_kd": MIT_CFG["kd"],
}

# Camera config
HEAD_CAMERA_PARAMS = {
    "serial_number": "P050HYX5410E1A001",
    "exposure": 16000,
    "gain": 100,
    "frame_rate": 30,
    "sens_ts": True,
}
LEFT_CAMERA_PARAMS = {
    "serial_number": "P050HYX5410E1A001",
    "exposure": 16000,
    "gain": 100,
    "frame_rate": 30,
    "sens_ts": True,
}
RIGHT_CAMERA_PARAMS = {
    "serial_number": "P050HYX5410E1A001",
    "exposure": 16000,
    "gain": 100,
    "frame_rate": 30,
    "sens_ts": True,
}

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ROBO_YOCO_DIR = f"{SCRIPT_DIR}/../../../hex_robo_yoco"
NODE_PARAMS_DICT = {
    # cli
    "e3_desktop_cli": {
        "name": "e3_desktop_cli",
        "node_path":
        f"{HEX_ROBO_YOCO_DIR}/../examples/basic/e3_desktop/cli.py",
        "cfg_path":
        f"{HEX_ROBO_YOCO_DIR}/../examples/basic/e3_desktop/cli.json",
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
                    "port": SRV_CFG["mujoco_port"]
                },
                "left_robot_net": {
                    "port": SRV_CFG["left_robot_port"]
                },
                "right_robot_net": {
                    "port": SRV_CFG["right_robot_port"]
                },
                "head_camera_net": {
                    "port": SRV_CFG["head_camera_port"]
                },
                "left_camera_net": {
                    "port": SRV_CFG["left_camera_port"]
                },
                "right_camera_net": {
                    "port": SRV_CFG["right_camera_port"]
                },
            },
        },
    },
}

if YOCO["use_sim"]:
    NODE_PARAMS_DICT["mujoco_e3_desktop_srv"] = {
        "name": "mujoco_e3_desktop_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["mujoco_e3_desktop"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["mujoco_e3_desktop"],
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": SRV_CFG["mujoco_port"],
            },
            "params": MUJOCO_PARAMS,
        },
    }
else:
    NODE_PARAMS_DICT["left_robot_e3_desktop_srv"] = {
        "name": "left_robot_e3_desktop_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],
        "cfg": {
            "net": {
                "port": SRV_CFG["left_robot_port"],
            },
            "params": LEFT_ROBOT_PARAMS,
        },
    }
    NODE_PARAMS_DICT["right_robot_e3_desktop_srv"] = {
        "name": "right_robot_e3_desktop_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],
        "cfg": {
            "net": {
                "port": SRV_CFG["right_robot_port"],
            },
            "params": RIGHT_ROBOT_PARAMS,
        },
    }
    if YOCO["cam_type"][0] == "berxel":
        NODE_PARAMS_DICT["head_camera_e3_desktop_srv"] = {
            "name": "head_camera_e3_desktop_srv",
            "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_berxel"],
            "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_berxel"],
            "cfg": {
                "net": {
                    "port": SRV_CFG["head_camera_port"],
                },
                "params": HEAD_CAMERA_PARAMS,
            },
        }
    if YOCO["cam_type"][1] == "berxel":
        NODE_PARAMS_DICT["left_camera_e3_desktop_srv"] = {
            "name": "left_camera_e3_desktop_srv",
            "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_berxel"],
            "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_berxel"],
            "cfg": {
                "net": {
                    "port": SRV_CFG["left_camera_port"],
                },
                "params": LEFT_CAMERA_PARAMS,
            },
        }
    if YOCO["cam_type"][2] == "berxel":
        NODE_PARAMS_DICT["right_camera_e3_desktop_srv"] = {
            "name": "right_camera_e3_desktop_srv",
            "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_berxel"],
            "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_berxel"],
            "cfg": {
                "net": {
                    "port": SRV_CFG["right_camera_port"],
                },
                "params": RIGHT_CAMERA_PARAMS,
            },
        }


def get_node_cfgs(node_params_dict: dict = NODE_PARAMS_DICT,
                  launch_args: dict | None = None):
    return HexNodeConfig.parse_node_params_dict(
        node_params_dict,
        NODE_PARAMS_DICT,
    )


def main():
    node_cfgs = get_node_cfgs()
    launch = HexLaunch(node_cfgs)
    launch.run()


if __name__ == '__main__':
    main()
