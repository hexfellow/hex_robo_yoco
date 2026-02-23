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
    HEX_LOG_LEVEL,
    hex_log,
)
from hex_robo_utils import (
    HexDynUtil as DynUtil,
    HexRate,
)


def wait_client_working(client, timeout: float = 5.0) -> bool:
    for _ in range(int(timeout * 10)):
        if client.is_working():
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


def calc_tau_comp(
    q_cur: np.ndarray,
    dq_cur: np.ndarray,
    dyn_util: DynUtil,
    dofs: dict,
):
    q_arm = q_cur[:dofs["robot_arm"]]
    dq_arm = dq_cur[:dofs["robot_arm"]]

    tau_comp = np.zeros(dofs["sum"])
    _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(q_arm, dq_arm)
    tau_comp[:dofs["robot_arm"]] = c_mat @ dq_arm + g_vec
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
        mit_cfg = cfg["mit_cfg"]
        mit_kp = np.array(mit_cfg["kp"])
        mit_kd = np.array(mit_cfg["kd"])
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

    # wait for yoco client to work
    if not wait_client_working(client):
        hex_log(HEX_LOG_LEVEL["err"], "yoco client is not working")
        return

    # parameters
    dof_arr = client.get_dofs()
    dofs = {
        "robot_arm": dof_arr[0],
        "robot_gripper": dof_arr[1] if len(dof_arr) > 1 else None,
        "sum": dof_arr.sum(),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")
    limits = client.get_limits()
    hex_log(HEX_LOG_LEVEL["info"], f"limits: {limits.shape}")

    # get cam state and intri
    cam_state = client.get_cam_state()
    has_cam = cam_state["use_rgb"] or cam_state["use_depth"]
    if has_cam:
        intri = client.get_intri()
        hex_log(HEX_LOG_LEVEL["info"], f"intri: {intri}")

    # get states, rgb and depth, and set cmds
    q_tar = np.array(
        [0.0, -0.0205679922, 2.57081467, -0.978840246, 0.0, 0.0, 0.5])
    rate = HexRate(1000)
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
            rate.sleep()

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
                tau_comp = calc_tau_comp(
                    q_cur,
                    dq_cur,
                    dyn_util,
                    dofs,
                )
                # ((q_tar_0, dq_tar_0, tau_comp_0, kp_0, kd_0), (q_tar_1, dq_tar_1, tau_comp_1, kp_1, kd_1), ...)
                cmds = np.vstack(
                    (q_tar, np.zeros(dofs["sum"]), tau_comp, mit_kp, mit_kd)).T
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

            if cam_state["use_depth"]:
                depth_hdr, depth_img = client.get_depth()
                if depth_hdr is not None:
                    depth_cmap = depth_to_cmap(depth_img)
                    cv2.imshow("depth_cmap", depth_cmap)

            if cam_state["use_rgb"]:
                rgb_hdr, rgb_img = client.get_rgb()
                if rgb_hdr is not None:
                    cv2.imshow("rgb_img", rgb_img)

            if has_cam:
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

            elapsed_time = (time.perf_counter_ns() - loop_start_time) / 1e6
            if elapsed_time > 2.0:
                err_dict["loop"] += 1

            if (i + 1) % 1_000 == 0:
                hex_log(HEX_LOG_LEVEL["info"], f"{i + 1}/{test_num}")

        print("stop moving")
        for _ in range(10):
            rate.sleep()

            if (q_cur is not None) and (dq_cur is not None):
                tau_comp = calc_tau_comp(
                    q_cur=q_cur,
                    dq_cur=dq_cur,
                    dyn_util=dyn_util,
                    dofs=dofs,
                )
                cmds = np.vstack(
                    (q_tar, np.zeros(dofs["sum"]), tau_comp, mit_kp, mit_kd)).T
                _ = client.set_cmds(cmds)

    finally:
        cv2.destroyAllWindows()
        hex_log(HEX_LOG_LEVEL["info"], f"##### err_dict: #####")
        for key, value in err_dict.items():
            hex_log(HEX_LOG_LEVEL["info"], f"## {key}: {value}")


if __name__ == '__main__':
    main()
