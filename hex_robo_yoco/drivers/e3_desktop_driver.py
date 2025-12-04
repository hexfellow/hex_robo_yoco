#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-12-03
################################################################

import copy
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT
from importlib.util import find_spec

# YOCO config
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
            "headless": True,
        },
        "robot": {
            "mit_kp": [200.0, 200.0, 250.0, 150.0, 20.0, 20.0, 20.0],
            "mit_kd": [5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
            "arm_type": "archer_y6",
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
            "device_ip": "172.18.8.161",
            "device_port": 8439,
        },
        "right_robot": {
            "device_ip": "172.18.8.161",
            "device_port": 9439,
        },
        "head_camera": {},
        "left_camera": {},
        "right_camera": {},
    },
}

# Mujoco srv
MUJOCO_E3_DESKTOP_SRV = {
    "name": "mujoco_e3_desktop_srv",
    "node_path": HEX_ZMQ_SERVERS_PATH_DICT["mujoco_e3_desktop"],
    "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["mujoco_e3_desktop"],
    "cfg": {
        "net": {
            "ip": "127.0.0.1",
            "port": YOCO["srv_port"]["mujoco_port"],
        },
        "params": {
            "states_rate": 1000,
            "img_rate": 30,
            "tau_ctrl": False,
            "headless": YOCO["params"]["mujoco"]["headless"],
            "sens_ts": True,
            "mit_kp": YOCO["params"]["robot"]["mit_kp"],
            "mit_kd": YOCO["params"]["robot"]["mit_kd"],
            "cam_type": YOCO["cam_type"],
        },
    },
}

# Robot srv
ROBOT_HEXARM_SRV = {
    "name": "robot_e3_desktop_srv",
    "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],
    "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],
    "cfg": {
        "net": {
            "port": YOCO["srv_port"]["left_robot_port"],
        },
        "params": {
            "device_ip": YOCO["device"]["left_robot"]["device_ip"],
            "device_port": YOCO["device"]["left_robot"]["device_port"],
            "control_hz": 1000,
            "sens_ts": True,
            "arm_type": YOCO["params"]["robot"]["arm_type"],
            "use_gripper": True,
            "mit_kp": YOCO["params"]["robot"]["mit_kp"],
            "mit_kd": YOCO["params"]["robot"]["mit_kd"],
        },
    },
}

# RGB srv
RGB_SRV = {
    "name": "camera_e3_desktop_srv",
    "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_rgb"],
    "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_rgb"],
    "cfg": {
        "net": {
            "port": YOCO["srv_port"]["head_camera_port"],
        },
        "params": {
            "cam_path": "/dev/video0",
            "resolution": YOCO["params"]["rgb"]["resolution"],
            "crop": YOCO["params"]["rgb"]["crop"],
            "exposure": YOCO["params"]["rgb"]["exposure"],
            "temperature": YOCO["params"]["rgb"]["temperature"],
            "frame_rate": 30,
            "sens_ts": True,
        },
    },
}

# Realsense srv
_HAS_REALSENSE = find_spec("pyrealsense2") is not None
if _HAS_REALSENSE:
    REALSENSE_SRV = {
        "name": "camera_archer_y6_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_realsense"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_realsense"],
        "cfg": {
            "net": {
                "port": YOCO["srv_port"]["head_camera_port"],
            },
            "params": {
                "serial_number": "243422073194",
                "resolution": YOCO["params"]["realsense"]["resolution"],
                "frame_rate": 30,
                "sens_ts": True,
            },
        },
    }

# Berxel srv
_HAS_BERXEL = find_spec("berxel_py_wrapper") is not None
if _HAS_BERXEL:
    BERXEL_SRV = {
        "name": "camera_e3_desktop_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_berxel"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_berxel"],
        "cfg": {
            "net": {
                "port": YOCO["srv_port"]["head_camera_port"],
            },
            "params": {
                "serial_number": "P100RYB4C03M2B322",
                "exposure": YOCO["params"]["berxel"]["exposure"],
                "gain": YOCO["params"]["berxel"]["gain"],
                "frame_rate": 30,
                "sens_ts": True,
            },
        },
    }


def set_mujoco_node_cfg(
    default_node_params_dict: dict,
    mujoco_params: dict,
    robot_params: dict,
):
    # mujoco
    default_node_params_dict["mujoco_e3_desktop_srv"]["cfg"]["params"][
        "headless"] = mujoco_params.get(
            "headless",
            MUJOCO_E3_DESKTOP_SRV["cfg"]["params"]["headless"],
        )
    # robot
    default_node_params_dict["mujoco_e3_desktop_srv"]["cfg"]["params"][
        "mit_kp"] = robot_params.get(
            "mit_kp",
            MUJOCO_E3_DESKTOP_SRV["cfg"]["params"]["mit_kp"],
        )
    default_node_params_dict["mujoco_e3_desktop_srv"]["cfg"]["params"][
        "mit_kd"] = robot_params.get(
            "mit_kd",
            MUJOCO_E3_DESKTOP_SRV["cfg"]["params"]["mit_kd"],
        )
    return default_node_params_dict


def set_robot_node_cfg(
    default_node_params_dict: dict,
    robot_params: dict,
    robot_device: dict,
    robot_name: str,
):
    # name
    default_node_params_dict[f"{robot_name}_robot_e3_desktop_srv"][
        "name"] = f"{robot_name}_robot_e3_desktop_srv"
    # params
    default_node_params_dict[f"{robot_name}_robot_e3_desktop_srv"]["cfg"][
        "params"]["arm_type"] = robot_params.get(
            "arm_type",
            ROBOT_HEXARM_SRV["cfg"]["params"]["arm_type"],
        )
    default_node_params_dict[f"{robot_name}_robot_e3_desktop_srv"]["cfg"][
        "params"]["mit_kp"] = robot_params.get(
            "mit_kp",
            ROBOT_HEXARM_SRV["cfg"]["params"]["mit_kp"],
        )
    default_node_params_dict[f"{robot_name}_robot_e3_desktop_srv"]["cfg"][
        "params"]["mit_kd"] = robot_params.get(
            "mit_kd",
            ROBOT_HEXARM_SRV["cfg"]["params"]["mit_kd"],
        )
    # device
    default_node_params_dict[f"{robot_name}_robot_e3_desktop_srv"]["cfg"][
        "params"]["device_ip"] = robot_device.get(
            "device_ip",
            ROBOT_HEXARM_SRV["cfg"]["params"]["device_ip"],
        )
    default_node_params_dict[f"{robot_name}_robot_e3_desktop_srv"]["cfg"][
        "params"]["device_port"] = robot_device.get(
            "device_port",
            ROBOT_HEXARM_SRV["cfg"]["params"]["device_port"],
        )
    return default_node_params_dict


def set_rgb_node_cfg(
    default_node_params_dict: dict,
    rgb_params: dict,
    rgb_device: dict,
    rgb_name: str,
):
    # name
    default_node_params_dict[f"{rgb_name}_camera_e3_desktop_srv"][
        "name"] = f"{rgb_name}_camera_e3_desktop_srv"
    # params
    default_node_params_dict[f"{rgb_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["resolution"] = rgb_params.get(
            "resolution",
            RGB_SRV["cfg"]["params"]["resolution"],
        )
    default_node_params_dict[f"{rgb_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["crop"] = rgb_params.get(
            "crop",
            RGB_SRV["cfg"]["params"]["crop"],
        )
    default_node_params_dict[f"{rgb_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["exposure"] = rgb_params.get(
            "exposure",
            RGB_SRV["cfg"]["params"]["exposure"],
        )
    default_node_params_dict[f"{rgb_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["temperature"] = rgb_params.get(
            "temperature",
            RGB_SRV["cfg"]["params"]["temperature"],
        )
    # device
    default_node_params_dict[f"{rgb_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["cam_path"] = rgb_device.get(
            "cam_path",
            RGB_SRV["cfg"]["params"]["cam_path"],
        )
    return default_node_params_dict


def set_realsense_node_cfg(
    default_node_params_dict: dict,
    realsense_params: dict,
    realsense_device: dict,
    realsense_name: str,
):
    # name
    default_node_params_dict[f"{realsense_name}_camera_e3_desktop_srv"][
        "name"] = f"{realsense_name}_camera_e3_desktop_srv"
    # params
    default_node_params_dict[f"{realsense_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["resolution"] = realsense_params.get(
            "resolution",
            REALSENSE_SRV["cfg"]["params"]["resolution"],
        )
    # device
    default_node_params_dict[f"{realsense_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["serial_number"] = realsense_device.get(
            "serial_number",
            REALSENSE_SRV["cfg"]["params"]["serial_number"],
        )
    return default_node_params_dict


def set_berxel_node_cfg(
    default_node_params_dict: dict,
    berxel_params: dict,
    berxel_device: dict,
    berxel_name: str,
):
    # name
    default_node_params_dict[f"{berxel_name}_camera_e3_desktop_srv"][
        "name"] = f"{berxel_name}_camera_e3_desktop_srv"
    # params
    default_node_params_dict[f"{berxel_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["exposure"] = berxel_params.get(
            "exposure",
            BERXEL_SRV["cfg"]["params"]["exposure"],
        )
    default_node_params_dict[f"{berxel_name}_camera_e3_desktop_srv"]["cfg"][
        "params"]["gain"] = berxel_params.get(
            "gain",
            BERXEL_SRV["cfg"]["params"]["gain"],
        )
    # device
    default_node_params_dict["camera_archer_y6_srv"]["cfg"]["params"][
        "serial_number"] = berxel_device.get(
            "serial_number",
            BERXEL_SRV["cfg"]["params"]["serial_number"],
        )
    return default_node_params_dict


def get_node_cfgs(node_params_dict: dict = {}, launch_args: dict = YOCO):
    default_node_params_dict = {}
    use_sim = launch_args.get("use_sim", YOCO["use_sim"])
    cam_type = launch_args.get("cam_type", YOCO["cam_type"])
    srv_port = launch_args.get("srv_port", YOCO["srv_port"])
    params = launch_args.get("params", YOCO["params"])
    device = launch_args.get("device", YOCO["device"])
    if use_sim:
        default_node_params_dict[
            "mujoco_e3_desktop_srv"] = MUJOCO_E3_DESKTOP_SRV
        default_node_params_dict["mujoco_e3_desktop_srv"]["cfg"]["params"][
            "cam_type"] = cam_type
        default_node_params_dict["mujoco_e3_desktop_srv"]["cfg"]["net"][
            "port"] = srv_port.get("mujoco_port",
                                   YOCO["srv_port"]["mujoco_port"])
        default_node_params_dict = set_mujoco_node_cfg(
            default_node_params_dict,
            params.get("mujoco", YOCO["params"]["mujoco"]),
            params.get("robot", YOCO["params"]["robot"]),
        )
    else:
        for name in ["left", "right"]:
            default_node_params_dict[
                f"{name}_robot_e3_desktop_srv"] = copy.deepcopy(
                    ROBOT_HEXARM_SRV)
            default_node_params_dict[f"{name}_robot_e3_desktop_srv"]["cfg"][
                "net"]["port"] = srv_port.get(
                    f"{name}_robot_port",
                    YOCO["srv_port"][f"{name}_robot_port"])
            default_node_params_dict = set_robot_node_cfg(
                default_node_params_dict,
                params.get("robot", YOCO["params"]["robot"]),
                device.get(f"{name}_robot", YOCO["device"][f"{name}_robot"]),
                name,
            )
        # cam_type: empty, rgb, realsense, berxel
        for cam, name in zip(cam_type, ["head", "left", "right"]):
            if cam == "rgb":
                default_node_params_dict[
                    f"{name}_camera_e3_desktop_srv"] = copy.deepcopy(RGB_SRV)
                default_node_params_dict[f"{name}_camera_e3_desktop_srv"][
                    "cfg"]["net"]["port"] = srv_port.get(
                        f"{name}_camera_port",
                        YOCO["srv_port"][f"{name}_camera_port"])
                default_node_params_dict = set_rgb_node_cfg(
                    default_node_params_dict,
                    params.get("rgb", YOCO["params"]["rgb"]),
                    device.get(f"{name}_camera",
                               YOCO["device"][f"{name}_camera"]),
                    name,
                )
            elif cam == "realsense":
                if _HAS_REALSENSE:
                    default_node_params_dict[
                        f"{name}_camera_e3_desktop_srv"] = copy.deepcopy(
                            REALSENSE_SRV)
                    default_node_params_dict[f"{name}_camera_e3_desktop_srv"][
                        "cfg"]["net"]["port"] = srv_port.get(
                            f"{name}_camera_port",
                            YOCO["srv_port"][f"{name}_camera_port"])
                    default_node_params_dict = set_realsense_node_cfg(
                        default_node_params_dict,
                        params.get("realsense", YOCO["params"]["realsense"]),
                        device.get(f"{name}_camera",
                                   YOCO["device"][f"{name}_camera"]),
                        name,
                    )
            elif cam == "berxel":
                if _HAS_BERXEL:
                    default_node_params_dict[
                        f"{name}_camera_e3_desktop_srv"] = copy.deepcopy(
                            BERXEL_SRV)
                    default_node_params_dict[f"{name}_camera_e3_desktop_srv"][
                        "cfg"]["net"]["port"] = srv_port.get(
                            f"{name}_camera_port",
                            YOCO["srv_port"][f"{name}_camera_port"])
                    default_node_params_dict = set_berxel_node_cfg(
                        default_node_params_dict,
                        params.get("berxel", YOCO["params"]["berxel"]),
                        device.get(f"{name}_camera",
                                   YOCO["device"][f"{name}_camera"]),
                        name,
                    )
            elif cam == "empty":
                pass
            else:
                raise ValueError(f"unknown camera type: {cam}")

    return HexNodeConfig.parse_node_params_dict(
        node_params_dict,
        default_node_params_dict,
    )


def main():
    node_cfgs = get_node_cfgs()
    launch = HexLaunch(node_cfgs)
    launch.run()


if __name__ == '__main__':
    main()
