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
from hex_zmq_servers import HexCamBerxelClient


class HexYocoArcherY6:

    def __init__(self, yoco_config: dict, net_config: dict):
        try:
            use_sim = yoco_config["use_sim"]
            use_cam = yoco_config["use_cam"]
            if use_sim:
                mujoco_net_config = net_config["mujoco_net"]
            else:
                robot_net_config = net_config["robot_net"]
                camera_net_config = net_config["camera_net"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(f"Missing key: [{missing_key}] in yoco_config")

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

    def get_obj_pose(self):
        if self.__use_sim:
            return self.__clients["mujoco"].get_states("obj")
        else:
            raise ValueError("`get_obj_pose` is not supported in real mode")

    def seq_clear(self):
        if self.__use_sim:
            return self.__clients["mujoco"].seq_clear()
        else:
            return self.__clients["robot"].seq_clear()

    def get_dofs(self):
        if self.__use_sim:
            return self.__clients["mujoco"].get_dofs()
        else:
            return self.__clients["robot"].get_dofs()

    def get_limits(self):
        if self.__use_sim:
            return self.__clients["mujoco"].get_limits()
        else:
            return self.__clients["robot"].get_limits()

    def get_states(self):
        if self.__use_sim:
            return self.__clients["mujoco"].get_states('robot')
        else:
            return self.__clients["robot"].get_states()

    def set_cmds(self, cmds: np.ndarray) -> bool:
        if self.__use_sim:
            return self.__clients["mujoco"].set_cmds(cmds)
        else:
            return self.__clients["robot"].set_cmds(cmds)

    def get_intri(self):
        if self.__use_cam:
            if self.__use_sim:
                return self.__clients["mujoco"].get_intri()
            else:
                return self.__clients["camera"].get_intri()
        else:
            raise ValueError("`get_intri` is not supported without `use_cam`")

    def get_rgb(self):
        if self.__use_cam:
            if self.__use_sim:
                return self.__clients["mujoco"].get_rgb()
            else:
                return self.__clients["camera"].get_rgb()
        else:
            raise ValueError("`get_rgb` is not supported without `use_cam`")

    def get_depth(self):
        if self.__use_cam:
            if self.__use_sim:
                return self.__clients["mujoco"].get_depth()
            else:
                return self.__clients["camera"].get_depth()
        else:
            raise ValueError("`get_depth` is not supported without `use_cam`")
