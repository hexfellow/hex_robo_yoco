#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time
from hex_robo_yoco import HexYocoArcherY6

import cv2
import numpy as np
from hex_zmq_servers import (
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
)
from hex_robo_utils import HexDynUtil as DynUtil
from hex_robo_utils import HexCtrlUtilMitJoint as CtrlUtil


def wait_client_working(client, timeout: float = 5.0) -> bool:
    for _ in range(int(timeout * 10)):
        working = client.is_working()
        if working is not None and working["cmd"] == "is_working_ok":
            if hasattr(client, "seq_clear"):
                client.seq_clear()
            return True
        else:
            time.sleep(0.1)
    return False


def depth_to_cmap(depth_img: np.ndarray):
    depth_values = depth_img.astype(np.float32)
    depth_norm = np.clip((depth_values - 70) / (1000 - 70), 0.0, 1.0)
    depth_u8 = (depth_norm * 255.0).astype(np.uint8)
    depth_cmap = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
    return depth_cmap


def cal_tau_comp(
    q_cur: np.ndarray,
    dq_cur: np.ndarray,
    dyn_util: DynUtil,
    dofs: int,
    use_gripper: bool,
):
    tau_comp = np.zeros(dofs)
    q_arm = q_cur[:-1] if use_gripper else q_cur
    dq_arm = dq_cur[:-1] if use_gripper else dq_cur
    _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(q_arm, dq_arm)
    if use_gripper:
        tau_comp[:-1] = c_mat @ dq_arm + g_vec
    else:
        tau_comp = c_mat @ dq_arm + g_vec
    return tau_comp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        yoco_config = cfg["yoco"]
        net_config = cfg["net"]
        model_path = cfg["model_path"]
        use_gripper = cfg["use_gripper"]
        mit_kp = cfg["mit_cfg"]["kp"] if use_gripper else cfg["mit_cfg"][
            "kp"][:-1]
        mit_kd = cfg["mit_cfg"]["kd"] if use_gripper else cfg["mit_cfg"][
            "kd"][:-1]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(
            f"archer_y6_config is not valid, missing key: {missing_key}")

    # init
    client = HexYocoArcherY6(yoco_config=yoco_config, net_config=net_config)
    yoco_config = client.get_yoco_config()
    dyn_util = DynUtil(
        model_path=model_path,
        end_pose=np.array(
            [0.0, 0.0, 0.187, 0.7071068, 0.0, -0.7071068, 0.0],
            dtype=np.float64,
        ),
    )
    ctrl_util = CtrlUtil()

    # wait for yoco client to work
    if not wait_client_working(client):
        hex_log(HEX_LOG_LEVEL["err"], "yoco client is not working")
        return

    # get dofs, limits and intri
    dofs = client.get_dofs()
    limits = client.get_limits()
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")
    hex_log(HEX_LOG_LEVEL["info"], f"limits: {limits}")
    if yoco_config["use_cam"]:
        intri = client.get_intri()
        hex_log(HEX_LOG_LEVEL["info"], f"intri: {intri}")

    # get states, rgb and depth, and set cmds
    q_tar = np.array(
        [0.0, -0.0205679922, 2.57081467, -0.978840246, 0.0, 0.0, 0.5])
    rate = HexRate(500)
    test_num = 10_000
    err_dict = {
        "get_states": 0,
        "calc_cmds": 0,
        "set_cmds": 0,
        "loop": 0,
    }
    try:
        q_cur = None
        dq_cur = None
        for i in range(test_num):
            loop_start_time = time.perf_counter_ns()

            get_states_start_time = time.perf_counter_ns()
            robot_states_hdr, robot_states = client.get_states()
            get_states_elapsed_time = (time.perf_counter_ns() -
                                       get_states_start_time) / 1e6
            if get_states_elapsed_time > 1.0:
                err_dict["get_states"] += 1
            if robot_states_hdr is not None:
                q_cur = robot_states[:, 0]
                dq_cur = robot_states[:, 1]

            # hex_log(HEX_LOG_LEVEL["info"], f"cmds: {cmds}")
            if q_cur is not None and dq_cur is not None:
                calc_cmds_start_time = time.perf_counter_ns()
                tau_comp = cal_tau_comp(
                    q_cur,
                    dq_cur,
                    dyn_util,
                    dofs,
                    use_gripper,
                )
                cmds = ctrl_util(
                    kp=mit_kp,
                    kd=mit_kd,
                    q_tar=q_tar,
                    dq_tar=np.zeros(dofs),
                    q_cur=q_cur,
                    dq_cur=dq_cur,
                    tau_comp=tau_comp,
                )
                calc_cmds_elapsed_time = (time.perf_counter_ns() -
                                          calc_cmds_start_time) / 1e6
                if calc_cmds_elapsed_time > 0.2:
                    err_dict["calc_cmds"] += 1
                set_cmds_start_time = time.perf_counter_ns()
                _ = client.set_cmds(cmds)
                set_cmds_elapsed_time = (time.perf_counter_ns() -
                                         set_cmds_start_time) / 1e6
                if set_cmds_elapsed_time > 1.0:
                    err_dict["set_cmds"] += 1

            if yoco_config["use_cam"]:
                depth_hdr, depth_img = client.get_depth()
                if depth_hdr is not None:
                    depth_cmap = depth_to_cmap(depth_img)
                    cv2.imshow("depth_cmap", depth_cmap)

                rgb_hdr, rgb_img = client.get_rgb()
                if rgb_hdr is not None:
                    cv2.imshow("rgb_img", rgb_img)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

            elapsed_time = (time.perf_counter_ns() - loop_start_time) / 1e6
            if elapsed_time > 2.0:
                err_dict["loop"] += 1

            if (i + 1) % 1_000 == 0:
                hex_log(HEX_LOG_LEVEL["info"], f"{i + 1}/{test_num}")

            rate.sleep()
    finally:
        cv2.destroyAllWindows()
        hex_log(HEX_LOG_LEVEL["info"], f"##### err_dict: #####")
        for key, value in err_dict.items():
            hex_log(HEX_LOG_LEVEL["info"], f"## {key}: {value}")


if __name__ == '__main__':
    main()
