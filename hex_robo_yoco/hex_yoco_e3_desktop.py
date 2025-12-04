#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-11-21
################################################################

import numpy as np
from hex_zmq_servers import HexMujocoE3DesktopClient
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


class HexYocoE3Desktop:

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
            for idx in range(len(cam_type)):
                if cam_type[idx] == "berxel" and not _HAS_BERXEL:
                    print(
                        "`berxel_py_wrapper` not found, setting cam_type to empty"
                    )
                    cam_type[idx] = "empty"
                elif cam_type[idx] == "realsense" and not _HAS_REALSENSE:
                    print(
                        "`pyrealsense2` not found, setting cam_type to empty")
                    cam_type[idx] = "empty"
            if use_sim:
                mujoco_net_config = net_config[f"{self.__prefix}mujoco_net"]
            else:
                left_robot_net_config = net_config[
                    f"{self.__prefix}left_robot_net"]
                right_robot_net_config = net_config[
                    f"{self.__prefix}right_robot_net"]
                camera_net_config = {}
                for idx, cam_name in enumerate(["head", "left", "right"]):
                    camera_net_config[cam_name] = net_config[
                        f"{self.__prefix}{cam_name}_camera_net"] if cam_type[
                            idx] != "empty" else None
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"Missing key: [{missing_key}] in yoco_config or net_config")

        self.__use_sim = use_sim
        self.__cam_type = cam_type
        self.__use_rgb, self.__use_depth, self.__rgb_shape, self.__depth_shape = {}, {}, {}, {}
        for idx, cam_name in enumerate(["head", "left", "right"]):
            (self.__use_rgb[cam_name], self.__use_depth[cam_name]), (
                self.__rgb_shape[cam_name],
                self.__depth_shape[cam_name]) = CAMERA_CONFIG.get(
                    cam_type[idx], [(False, False), (None, None)])

        self.__clients = {}
        if self.__use_sim:
            self.__clients["mujoco"] = HexMujocoE3DesktopClient(
                net_config=mujoco_net_config,
                recv_config={
                    "head_rgb": self.__use_rgb["head"],
                    "left_rgb": self.__use_rgb["left"],
                    "right_rgb": self.__use_rgb["right"],
                    "head_depth": self.__use_depth["head"],
                    "left_depth": self.__use_depth["left"],
                    "right_depth": self.__use_depth["right"],
                    "obj": False,
                },
            )
        else:
            self.__clients["left_robot"] = HexRobotHexarmClient(
                net_config=left_robot_net_config)
            self.__clients["right_robot"] = HexRobotHexarmClient(
                net_config=right_robot_net_config)
            for cam_name in ["head", "left", "right"]:
                self.__clients[cam_name] = None
                if cam_type[idx] == "berxel":
                    self.__clients[f"{cam_name}_camera"] = HexCamBerxelClient(
                        net_config=camera_net_config[cam_name])
                elif cam_type[idx] == "realsense":
                    self.__clients[
                        f"{cam_name}_camera"] = HexCamRealsenseClient(
                            net_config=camera_net_config[cam_name])
                elif cam_type[idx] == "rgb":
                    self.__clients[f"{cam_name}_camera"] = HexCamRGBClient(
                        net_config=camera_net_config[cam_name])

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
            left_working = self.__clients["left_robot"].is_working()
            right_working = self.__clients["right_robot"].is_working()
            camera_working = True
            for cam_name in ["head", "left", "right"]:
                camera_working = camera_working and (
                    self.__clients[cam_name].is_working()
                    if self.__clients[cam_name] is not None else True)
            return left_working and right_working and camera_working

    def reset(self):
        if self.__use_sim:
            return self.__clients["mujoco"].reset()
        else:
            raise ValueError("`reset` is not supported in real mode")

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

    def get_states(self, robot_name: str, newest: bool = False):
        if robot_name not in ["left", "right"]:
            raise ValueError(f"robot_name must be in ['left', 'right']")

        if self.__use_sim:
            return self.__clients["mujoco"].get_states(robot_name=robot_name,
                                                       newest=newest)
        else:
            robot_key = None
            if robot_name == "left":
                robot_key = "left_robot"
            elif robot_name == "right":
                robot_key = "right_robot"
            else:
                raise ValueError(f"Invalid robot name: [{robot_name}]")
            return self.__clients[robot_key].get_states(newest=newest)

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
        use_cam = False
        for cam_name in ["head", "left", "right"]:
            use_cam = use_cam or self.__use_rgb[cam_name] or self.__use_depth[
                cam_name]
        if use_cam:
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
                    "head":
                    self.__clients["head_camera"].get_intri()[1] if
                    self.__clients["head_camera"] is not None else np.zeros(4),
                    "left":
                    self.__clients["left_camera"].get_intri()[1] if
                    self.__clients["left_camera"] is not None else np.zeros(4),
                    "right":
                    self.__clients["right_camera"].get_intri()[1]
                    if self.__clients["right_camera"] is not None else
                    np.zeros(4),
                }
        else:
            raise ValueError(
                f"`get_intri` is not supported with type {self.__cam_type}")

    def get_rgb(self, camera_name: str, newest: bool = False):
        if camera_name not in ["head", "left", "right"]:
            raise ValueError(
                f"camera_name must be in ['head', 'left', 'right']")

        if self.__use_rgb[camera_name]:
            if self.__use_sim:
                return self.__clients["mujoco"].get_rgb(
                    camera_name=camera_name, newest=newest)
            else:
                return self.__clients[f"{camera_name}_camera"].get_rgb(
                    newest=newest)
        else:
            raise ValueError(
                f"`get_rgb` is not supported with type {self.__cam_type[camera_name]}"
            )

    def get_depth(self, camera_name: str, newest: bool = False):
        if camera_name not in ["head", "left", "right"]:
            raise ValueError(
                f"camera_name must be in ['head', 'left', 'right']")

        if self.__use_depth[camera_name]:
            if self.__use_sim:
                return self.__clients["mujoco"].get_depth(
                    camera_name=camera_name, newest=newest)
            else:
                return self.__clients[f"{camera_name}_camera"].get_depth(
                    newest=newest)
        else:
            raise ValueError(
                f"`get_depth` is not supported with type {self.__cam_type[camera_name]}"
            )
