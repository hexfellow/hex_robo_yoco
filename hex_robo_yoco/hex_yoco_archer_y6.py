#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-11-21
################################################################

import time
import numpy as np
from hex_zmq_servers import HexMujocoArcherY6Client
from hex_zmq_servers import HexRobotHexarmClient

BERXEL_CAMERA = True
try:
    from hex_zmq_servers import HexCamBerxelClient
except ImportError:
    BERXEL_CAMERA = False


class HexYocoArcherY6:

    def __init__(self, yoco_config: dict, net_config: dict):
        try:
            use_sim = yoco_config["use_sim"]
            if BERXEL_CAMERA:
                use_cam = yoco_config["use_cam"]
            else:
                print("HexCamBerxelClient not found, setting use_cam to False")
                use_cam = False
            if use_sim:
                mujoco_net_config = net_config["mujoco_net"]
            else:
                robot_net_config = net_config["robot_net"]
                camera_net_config = net_config["camera_net"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"Missing key: [{missing_key}] in yoco_config or net_config")

        self.__use_sim = use_sim
        self.__use_cam = use_cam

        self.__clients = {}
        if self.__use_sim:
            self.__clients["mujoco"] = HexMujocoArcherY6Client(
                net_config=mujoco_net_config)
        else:
            self.__clients["robot"] = HexRobotHexarmClient(
                net_config=robot_net_config)
            self.__clients["camera"] = HexCamBerxelClient(
                net_config=camera_net_config) if self.__use_cam else None

    def __del__(self):
        for client in self.__clients.values():
            if client is not None:
                client.close()

    def get_yoco_config(self):
        return {
            "use_sim": self.__use_sim,
            "use_cam": self.__use_cam,
        }

    def is_working(self):
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].is_working()
        else:
            robot_working = self.__clients["robot"].is_working()
            camera_working = self.__clients["camera"].is_working(
            ) if self.__clients["camera"] is not None else True
            return robot_working and camera_working

    def reset(self):
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].reset()
        else:
            raise ValueError("`reset` is not supported in real mode")

    def get_obj_pose(self):
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].get_states("obj")
        else:
            raise ValueError("`get_obj_pose` is not supported in real mode")

    def seq_clear(self):
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].seq_clear()
        else:
            return self.__clients["robot"].seq_clear()

    def get_dofs(self):
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].get_dofs()[0]
        else:
            return self.__clients["robot"].get_dofs()[0]

    def get_limits(self):
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].get_limits()[0].reshape(-1, 1, 2)
        else:
            return self.__clients["robot"].get_limits()[0]

    def get_states(self):
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].get_states('robot')
        else:
            return self.__clients["robot"].get_states()

    def set_cmds(self, cmds: np.ndarray) -> bool:
        time.sleep(0)
        if self.__use_sim:
            return self.__clients["mujoco"].set_cmds(cmds)
        else:
            return self.__clients["robot"].set_cmds(cmds)

    def get_intri(self):
        time.sleep(0)
        if self.__use_cam:
            if self.__use_sim:
                _, intri_array = self.__clients["mujoco"].get_intri()
                return intri_array
            else:
                _, intri_array = self.__clients["camera"].get_intri()
                return intri_array
        else:
            raise ValueError("`get_intri` is not supported without `use_cam`")

    def get_rgb(self):
        time.sleep(0)
        if self.__use_cam:
            if self.__use_sim:
                return self.__clients["mujoco"].get_rgb()
            else:
                return self.__clients["camera"].get_rgb()
        else:
            raise ValueError("`get_rgb` is not supported without `use_cam`")

    def get_depth(self):
        time.sleep(0)
        if self.__use_cam:
            if self.__use_sim:
                return self.__clients["mujoco"].get_depth()
            else:
                return self.__clients["camera"].get_depth()
        else:
            raise ValueError("`get_depth` is not supported without `use_cam`")
