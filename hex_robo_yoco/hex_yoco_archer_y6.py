#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-11-21
################################################################

import numpy as np
from hex_zmq_servers import HexMujocoArcherY6Client
from hex_zmq_servers import HexRobotHexarmClient
from hex_zmq_servers import HexCamRGBClient

from importlib.util import find_spec

_HAS_BERXEL = find_spec("berxel_py_wrapper") is not None
_HAS_REALSENSE = find_spec("pyrealsense2") is not None
if _HAS_BERXEL:
    from hex_zmq_servers import HexCamBerxelClient
if _HAS_REALSENSE:
    from hex_zmq_servers import HexCamRealsenseClient

CAMERA_CONFIG = {
    "empty": [(False, False), (None, None)],
    "rgb": [(True, False), ((480, 640, 3), None)],
    "berxel": [(True, True), ((400, 640, 3), (400, 640))],
    "realsense": [(True, True), ((480, 640, 3), (480, 640))],
}


class HexYocoArcherY6:

    def __init__(
        self,
        yoco_config: dict,
        net_config: dict,
        prefix: str = "",
    ):
        self.__prefix = prefix
        if self.__prefix != "":
            self.__prefix = f"{self.__prefix}_"
        try:
            use_sim = yoco_config["use_sim"]
            cam_type = yoco_config["cam_type"]
            if cam_type == "berxel" and not _HAS_BERXEL:
                print(
                    "`berxel_py_wrapper` not found, setting cam_type to empty")
                cam_type = "empty"
            elif cam_type == "realsense" and not _HAS_REALSENSE:
                print("`pyrealsense2` not found, setting cam_type to empty")
                cam_type = "empty"
            if use_sim:
                mujoco_net_config = net_config[f"{self.__prefix}mujoco_net"]
            else:
                robot_net_config = net_config[f"{self.__prefix}robot_net"]
                camera_net_config = net_config[
                    f"{self.__prefix}camera_net"] if cam_type != "empty" else None
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"Missing key: [{missing_key}] in yoco_config or net_config")

        self.__use_sim = use_sim
        self.__cam_type = cam_type
        (self.__use_rgb,
         self.__use_depth), (self.__rgb_shape,
                             self.__depth_shape) = CAMERA_CONFIG.get(
                                 cam_type, [(False, False), (None, None)])

        self.__clients = {}
        if self.__use_sim:
            self.__clients["mujoco"] = HexMujocoArcherY6Client(
                net_config=mujoco_net_config,
                recv_config={
                    "rgb": self.__use_rgb,
                    "depth": self.__use_depth,
                    "obj": False,
                },
            )
        else:
            self.__clients["robot"] = HexRobotHexarmClient(
                net_config=robot_net_config)
            self.__clients["camera"] = None
            if cam_type == "berxel":
                self.__clients["camera"] = HexCamBerxelClient(
                    net_config=camera_net_config)
            elif cam_type == "realsense":
                self.__clients["camera"] = HexCamRealsenseClient(
                    net_config=camera_net_config)
            elif cam_type == "rgb":
                self.__clients["camera"] = HexCamRGBClient(
                    net_config=camera_net_config)

    def __del__(self):
        for client in self.__clients.values():
            if client is not None:
                client.close()

    def get_yoco_config(self):
        return {
            "use_sim": self.__use_sim,
            "cam_type": self.__cam_type,
        }

    def get_cam_state(self):
        return {
            "use_rgb": self.__use_rgb,
            "use_depth": self.__use_depth,
            "rgb_shape": self.__rgb_shape,
            "depth_shape": self.__depth_shape,
        }

    def is_working(self):
        if self.__use_sim:
            return self.__clients["mujoco"].is_working()
        else:
            robot_working = self.__clients["robot"].is_working()
            camera_working = self.__clients["camera"].is_working(
            ) if self.__clients["camera"] is not None else True
            return robot_working and camera_working

    def reset(self):
        if self.__use_sim:
            return self.__clients["mujoco"].reset()
        else:
            raise ValueError("`reset` is not supported in real mode")

    def seq_clear(self):
        if self.__use_sim:
            return self.__clients["mujoco"].seq_clear()
        else:
            return self.__clients["robot"].seq_clear()

    def get_dofs(self):
        if self.__use_sim:
            return self.__clients["mujoco"].get_dofs()[0]
        else:
            return self.__clients["robot"].get_dofs()[0]

    def get_limits(self):
        if self.__use_sim:
            return self.__clients["mujoco"].get_limits()[0].reshape(-1, 1, 2)
        else:
            return self.__clients["robot"].get_limits()[0]

    def get_states(self, newest: bool = False):
        if self.__use_sim:
            return self.__clients["mujoco"].get_states(newest=newest)
        else:
            return self.__clients["robot"].get_states(newest=newest)

    def set_cmds(self, cmds: np.ndarray) -> bool:
        if self.__use_sim:
            return self.__clients["mujoco"].set_cmds(cmds)
        else:
            return self.__clients["robot"].set_cmds(cmds)

    def get_intri(self):
        if self.__use_rgb or self.__use_depth:
            if self.__use_sim:
                _, intri_array = self.__clients["mujoco"].get_intri()
                return intri_array
            else:
                _, intri_array = self.__clients["camera"].get_intri()
                return intri_array
        else:
            raise ValueError(
                f"`get_intri` is not supported with type {self.__cam_type}")

    def get_rgb(self, newest: bool = False):
        if self.__use_rgb:
            if self.__use_sim:
                return self.__clients["mujoco"].get_rgb(newest=newest)
            else:
                return self.__clients["camera"].get_rgb(newest=newest)
        else:
            raise ValueError(
                f"`get_rgb` is not supported with type {self.__cam_type}")

    def get_depth(self, newest: bool = False):
        if self.__use_depth:
            if self.__use_sim:
                return self.__clients["mujoco"].get_depth(newest=newest)
            else:
                return self.__clients["camera"].get_depth(newest=newest)
        else:
            raise ValueError(
                f"`get_depth` is not supported with type {self.__cam_type}")
