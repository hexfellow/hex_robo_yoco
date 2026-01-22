#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time
from hex_robo_yoco import HexYocoE3Desktop

import cv2
import numpy as np
from hex_zmq_servers import (
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
)
from hex_robo_utils import HexDynUtil as DynUtil


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
    robot_name: str,
):
    assert robot_name in ["left",
                          "right"], f"robot_name must be in ['left', 'right']"

    q_arm = q_cur[:dofs[f"{robot_name}_arm"]]
    dq_arm = dq_cur[:dofs[f"{robot_name}_arm"]]

    tau_comp = np.zeros(dofs[f"{robot_name}_sum"])
    _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(q_arm, dq_arm)
    tau_comp[:dofs[f"{robot_name}_arm"]] = c_mat @ dq_arm + g_vec
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
            f"e3_desktop_mujoco_config is not valid, missing key: {missing_key}"
        )

    # init
    client = HexYocoE3Desktop(yoco_config=yoco_config, net_config=net_config)
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
        "left_arm": dof_arr[0],
        "left_gripper": dof_arr[1],
        "right_arm": dof_arr[2],
        "right_gripper": dof_arr[3],
        "left_sum": dof_arr[0] + dof_arr[1],
        "right_sum": dof_arr[2] + dof_arr[3],
    }
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")

    # get cam state and intri
    cam_state = client.get_cam_state()
    has_cam = False
    for cam_name in ["head", "left", "right"]:
        has_cam = has_cam or cam_state["use_rgb"][cam_name] or cam_state[
            "use_depth"][cam_name]
    if has_cam:
        intri = client.get_intri()
        hex_log(HEX_LOG_LEVEL["info"], f"intri: {intri}")

    # get states, rgb and depth, and set cmds
    q_tar_left = np.array(
        [-0.5, -0.0205679922, 2.57081467, -0.978840246, 0.5, 0.0, 0.5])
    q_tar_right = np.array(
        [0.5, -0.0205679922, 2.57081467, -0.978840246, -0.5, 0.0, 0.5])
    rate = HexRate(1000)
    test_num = 10_000
    err_dict = {
        "left_get_states": 0,
        "left_calc_cmds": 0,
        "left_set_cmds": 0,
        "right_get_states": 0,
        "right_calc_cmds": 0,
        "right_set_cmds": 0,
        "loop": 0,
    }
    try:
        q_cur_left = None
        dq_cur_left = None
        q_cur_right = None
        dq_cur_right = None
        for i in range(test_num):
            loop_start_time = time.perf_counter_ns()

            left_get_states_start_time = time.perf_counter_ns()
            left_states_hdr, left_states = client.get_states("left")
            left_get_states_elapsed_time = (time.perf_counter_ns() -
                                            left_get_states_start_time) / 1e6
            if left_get_states_elapsed_time > 1.0:
                err_dict["left_get_states"] += 1
            if left_states_hdr is not None:
                q_cur_left = left_states[:, 0]
                dq_cur_left = left_states[:, 1]

            right_get_states_start_time = time.perf_counter_ns()
            right_states_hdr, right_states = client.get_states("right")
            right_get_states_elapsed_time = (time.perf_counter_ns() -
                                             right_get_states_start_time) / 1e6
            if right_get_states_elapsed_time > 1.0:
                err_dict["right_get_states"] += 1
            if right_states_hdr is not None:
                q_cur_right = right_states[:, 0]
                dq_cur_right = right_states[:, 1]

            if (q_cur_left is not None) and (dq_cur_left is not None):
                left_calc_tau_comp_start_time = time.perf_counter_ns()
                tau_comp_left = calc_tau_comp(
                    q_cur=q_cur_left,
                    dq_cur=dq_cur_left,
                    dyn_util=dyn_util,
                    dofs=dofs,
                    robot_name="left",
                )
                # ((q_tar_0, dq_tar_0, tau_comp_0, kp_0, kd_0), (q_tar_1, dq_tar_1, tau_comp_1, kp_1, kd_1), ...)
                cmds_left = np.vstack((q_tar_left, np.zeros(dofs["left_sum"]),
                                       tau_comp_left, mit_kp, mit_kd)).T
                left_calc_tau_comp_elapsed_time = (
                    time.perf_counter_ns() -
                    left_calc_tau_comp_start_time) / 1e6
                if left_calc_tau_comp_elapsed_time > 0.2:
                    err_dict["left_calc_cmds"] += 1
                left_set_cmds_start_time = time.perf_counter_ns()
                _ = client.set_cmds(cmds_left, "left")
                left_set_cmds_elapsed_time = (time.perf_counter_ns() -
                                              left_set_cmds_start_time) / 1e6
                if left_set_cmds_elapsed_time > 1.0:
                    err_dict["left_set_cmds"] += 1

            if (q_cur_right is not None) and (dq_cur_right is not None):
                right_calc_tau_comp_start_time = time.perf_counter_ns()
                tau_comp_right = calc_tau_comp(
                    q_cur=q_cur_right,
                    dq_cur=dq_cur_right,
                    dyn_util=dyn_util,
                    dofs=dofs,
                    robot_name="right",
                )
                # ((q_tar_0, dq_tar_0, tau_comp_0, kp_0, kd_0), (q_tar_1, dq_tar_1, tau_comp_1, kp_1, kd_1), ...)
                cmds_right = np.vstack(
                    (q_tar_right, np.zeros(dofs["right_sum"]), tau_comp_right,
                     mit_kp, mit_kd)).T
                right_calc_tau_comp_elapsed_time = (
                    time.perf_counter_ns() -
                    right_calc_tau_comp_start_time) / 1e6
                if right_calc_tau_comp_elapsed_time > 0.2:
                    err_dict["right_calc_cmds"] += 1
                right_set_cmds_start_time = time.perf_counter_ns()
                _ = client.set_cmds(cmds_right, "right")
                right_set_cmds_elapsed_time = (time.perf_counter_ns() -
                                               right_set_cmds_start_time) / 1e6
                if right_set_cmds_elapsed_time > 1.0:
                    err_dict["right_set_cmds"] += 1

            if cam_state["use_depth"]["head"]:
                head_depth_hdr, head_depth_img = client.get_depth("head")
                if head_depth_hdr is not None:
                    head_depth_cmap = depth_to_cmap(head_depth_img)
                    cv2.imshow("head_depth_cmap", head_depth_cmap)

            if cam_state["use_rgb"]["head"]:
                head_rgb_hdr, head_rgb_img = client.get_rgb("head")
                if head_rgb_hdr is not None:
                    cv2.imshow("head_rgb_img", head_rgb_img)

            if cam_state["use_depth"]["left"]:
                left_depth_hdr, left_depth_img = client.get_depth("left")
                if left_depth_hdr is not None:
                    left_depth_cmap = depth_to_cmap(left_depth_img)
                    cv2.imshow("left_depth_cmap", left_depth_cmap)

            if cam_state["use_rgb"]["left"]:
                left_rgb_hdr, left_rgb_img = client.get_rgb("left")
                if left_rgb_hdr is not None:
                    cv2.imshow("left_rgb_img", left_rgb_img)

            if cam_state["use_depth"]["right"]:
                right_depth_hdr, right_depth_img = client.get_depth("right")
                if right_depth_hdr is not None:
                    right_depth_cmap = depth_to_cmap(right_depth_img)
                    cv2.imshow("right_depth_cmap", right_depth_cmap)

            if cam_state["use_rgb"]["right"]:
                right_rgb_hdr, right_rgb_img = client.get_rgb("right")
                if right_rgb_hdr is not None:
                    cv2.imshow("right_rgb_img", right_rgb_img)

            if has_cam:
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

            elapsed_time = (time.perf_counter_ns() - loop_start_time) / 1e6
            if elapsed_time > 2.0:
                err_dict["loop"] += 1

            if (i + 1) % 1_000 == 0:
                hex_log(HEX_LOG_LEVEL["info"], f"{i + 1}/{test_num}")

            rate.sleep()

        print("stop moving")
        for _ in range(10):
            if (q_cur_left is not None) and (dq_cur_left is not None):
                tau_comp_left = calc_tau_comp(
                    q_cur=q_cur_left,
                    dq_cur=dq_cur_left,
                    dyn_util=dyn_util,
                    dofs=dofs,
                    robot_name="left",
                )
                cmds_left = np.vstack((q_tar_left, np.zeros(dofs["left_sum"]),
                                       tau_comp_left, mit_kp, mit_kd)).T
                _ = client.set_cmds(cmds_left, "left")

            if (q_cur_right is not None) and (dq_cur_right is not None):
                tau_comp_right = calc_tau_comp(
                    q_cur=q_cur_right,
                    dq_cur=dq_cur_right,
                    dyn_util=dyn_util,
                    dofs=dofs,
                    robot_name="right",
                )
                cmds_right = np.vstack(
                    (q_tar_right, np.zeros(dofs["right_sum"]), tau_comp_right,
                     mit_kp, mit_kd)).T
                _ = client.set_cmds(cmds_right, "right")

            rate.sleep()

    finally:
        cv2.destroyAllWindows()
        hex_log(HEX_LOG_LEVEL["info"], f"##### err_dict: #####")
        for key, value in err_dict.items():
            hex_log(HEX_LOG_LEVEL["info"], f"##  {key}: {value}")


if __name__ == '__main__':
    main()
