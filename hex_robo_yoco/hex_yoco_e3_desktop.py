#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-11-21
################################################################

import time
import numpy as np
from hex_zmq_servers import HexMujocoE3DesktopClient
from hex_zmq_servers import HexRobotHexarmClient

BERXEL_CAMERA = True
try:
    from hex_zmq_servers import HexCamBerxelClient
except ImportError:
    BERXEL_CAMERA = False


class HexYocoE3Desktop:

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
                left_robot_net_config = net_config["left_robot_net"]
                right_robot_net_config = net_config["right_robot_net"]
                head_camera_net_config = net_config["head_camera_net"]
                left_camera_net_config = net_config["left_camera_net"]
                right_camera_net_config = net_config["right_camera_net"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"Missing key: [{missing_key}] in yoco_config or net_config")

        self.__use_sim = use_sim
        self.__use_cam = use_cam

        self.__clients = {}
        if self.__use_sim:
            self.__clients["mujoco"] = HexMujocoE3DesktopClient(
                net_config=mujoco_net_config)
        else:
            self.__clients["left_robot"] = HexRobotHexarmClient(
                net_config=left_robot_net_config)
            self.__clients["right_robot"] = HexRobotHexarmClient(
                net_config=right_robot_net_config)
            self.__clients["head_camera"] = HexCamBerxelClient(
                net_config=head_camera_net_config) if self.__use_cam else None
            self.__clients["left_camera"] = HexCamBerxelClient(
                net_config=left_camera_net_config) if self.__use_cam else None
            self.__clients["right_camera"] = HexCamBerxelClient(
                net_config=right_camera_net_config) if self.__use_cam else None

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
        if self.__use_sim:
            return self.__clients["mujoco"].is_working()
        else:
            left_working = self.__clients["left_robot"].is_working()
            right_working = self.__clients["right_robot"].is_working()
            head_camera_working = self.__clients["head_camera"].is_working(
            ) if self.__clients["head_camera"] is not None else True
            left_camera_working = self.__clients["left_camera"].is_working(
            ) if self.__clients["left_camera"] is not None else True
            right_camera_working = self.__clients["right_camera"].is_working(
            ) if self.__clients["right_camera"] is not None else True
            return left_working and right_working and left_camera_working and right_camera_working and head_camera_working

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
            clear_hdr = self.__clients["mujoco"].seq_clear()
            return {
                "left": clear_hdr,
                "right": clear_hdr,
            }
        else:
            return {
                "left": self.__clients["left_robot"].seq_clear(),
                "right": self.__clients["right_robot"].seq_clear(),
            }

    def get_dofs(self):
        if self.__use_sim:
            dofs_list = self.__clients["mujoco"].get_dofs()
            return {
                "left": dofs_list[0],
                "right": dofs_list[1],
            }
        else:
            return {
                "left": self.__clients["left_robot"].get_dofs()[0],
                "right": self.__clients["right_robot"].get_dofs()[0],
            }

    def get_limits(self):
        if self.__use_sim:
            limits_list = self.__clients["mujoco"].get_limits()
            return {
                "left": limits_list[0].reshape(-1, 1, 2),
                "right": limits_list[1].reshape(-1, 1, 2),
            }
        else:
            return {
                "left": self.__clients["left_robot"].get_limits()[0],
                "right": self.__clients["right_robot"].get_limits()[0],
            }

    def get_states(self, robot_name: str):
        if robot_name not in ["left", "right"]:
            raise ValueError(f"robot_name must be in ['left', 'right']")

        if self.__use_sim:
            return self.__clients["mujoco"].get_states(robot_name)
        else:
            robot_key = None
            if robot_name == "left":
                robot_key = "left_robot"
            elif robot_name == "right":
                robot_key = "right_robot"
            else:
                raise ValueError(f"Invalid robot name: [{robot_name}]")
            return self.__clients[robot_key].get_states()

    def set_cmds(self, cmds: np.ndarray, robot_name: str) -> bool:
        if robot_name not in ["left", "right"]:
            raise ValueError(f"robot_name must be in ['left', 'right']")

        if self.__use_sim:
            return self.__clients["mujoco"].set_cmds(cmds, robot_name)
        else:
            robot_key = None
            if robot_name == "left":
                robot_key = "left_robot"
            elif robot_name == "right":
                robot_key = "right_robot"
            else:
                raise ValueError(f"Invalid robot name: [{robot_name}]")
            return self.__clients[robot_key].set_cmds(cmds)

    def get_intri(self):
        if self.__use_cam:
            if self.__use_sim:
                _, intri_array = self.__clients["mujoco"].get_intri()
                print(f"intri_array: {intri_array}")
                return {
                    "head": intri_array[0],
                    "left": intri_array[1],
                    "right": intri_array[2],
                }
            else:
                return {
                    "head": self.__clients["head_camera"].get_intri()[1],
                    "left": self.__clients["left_camera"].get_intri()[1],
                    "right": self.__clients["right_camera"].get_intri()[1],
                }
        else:
            raise ValueError("`get_intri` is not supported without `use_cam`")

    def get_rgb(self, camera_name: str):
        if camera_name not in ["head", "left", "right"]:
            raise ValueError(
                f"camera_name must be in ['head', 'left', 'right']")

        if self.__use_cam:
            if self.__use_sim:
                return self.__clients["mujoco"].get_rgb(camera_name)
            else:
                camera_key = None
                if camera_name == "head":
                    camera_key = "head_camera"
                elif camera_name == "left":
                    camera_key = "left_camera"
                elif camera_name == "right":
                    camera_key = "right_camera"
                else:
                    raise ValueError(f"Invalid camera name: [{camera_name}]")
                return self.__clients[camera_key].get_rgb()
        else:
            raise ValueError("`get_rgb` is not supported without `use_cam`")

    def get_depth(self, camera_name: str):
        if camera_name not in ["head", "left", "right"]:
            raise ValueError(
                f"camera_name must be in ['head', 'left', 'right']")

        if self.__use_cam:
            if self.__use_sim:
                return self.__clients["mujoco"].get_depth(camera_name)
            else:
                camera_key = None
                if camera_name == "head":
                    camera_key = "head_camera"
                elif camera_name == "left":
                    camera_key = "left_camera"
                elif camera_name == "right":
                    camera_key = "right_camera"
                else:
                    raise ValueError(f"Invalid camera name: [{camera_name}]")
                return self.__clients[camera_key].get_depth()
        else:
            raise ValueError("`get_depth` is not supported without `use_cam`")
