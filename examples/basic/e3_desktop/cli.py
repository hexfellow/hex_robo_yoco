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
    q_tar_left = np.array(
        [-0.5, -0.0205679922, 2.57081467, -0.978840246, 0.5, 0.0, 0.5])
    q_tar_right = np.array(
        [0.5, -0.0205679922, 2.57081467, -0.978840246, -0.5, 0.0, 0.5])
    rate = HexRate(1000)
    try:
        q_cur_left = None
        dq_cur_left = None
        q_cur_right = None
        dq_cur_right = None
        while True:
            left_states_hdr, left_states = client.get_states("left")
            if left_states_hdr is not None:
                print(f"left_states_seq: {left_states_hdr['args']}")
                q_cur_left = left_states[:, 0]
                dq_cur_left = left_states[:, 1]

            if (q_cur_left is not None) and (dq_cur_left is not None):
                tau_comp_left = cal_tau_comp(
                    q_cur_left,
                    dq_cur_left,
                    dyn_util,
                    dofs["left"],
                    use_gripper,
                )
                cmds_left = ctrl_util(
                    kp=mit_kp,
                    kd=mit_kd,
                    q_tar=q_tar_left,
                    dq_tar=np.zeros(dofs["left"]),
                    q_cur=q_cur_left,
                    dq_cur=dq_cur_left,
                    tau_comp=tau_comp_left,
                )
                _ = client.set_cmds(cmds_left, "left")

            right_states_hdr, right_states = client.get_states("right")
            if right_states_hdr is not None:
                print(f"right_states_seq: {right_states_hdr['args']}")
                q_cur_right = right_states[:, 0]
                dq_cur_right = right_states[:, 1]

            if (q_cur_right is not None) and (dq_cur_right is not None):
                tau_comp_right = cal_tau_comp(
                    q_cur_right,
                    dq_cur_right,
                    dyn_util,
                    dofs["right"],
                    use_gripper,
                )
                cmds_right = ctrl_util(
                    kp=mit_kp,
                    kd=mit_kd,
                    q_tar=q_tar_right,
                    dq_tar=np.zeros(dofs["right"]),
                    q_cur=q_cur_right,
                    dq_cur=dq_cur_right,
                    tau_comp=tau_comp_right,
                )
                _ = client.set_cmds(cmds_right, "right")

            if yoco_config["use_cam"]:
                head_depth_hdr, head_depth_img = client.get_depth("head")
                if head_depth_hdr is not None:
                    head_depth_cmap = depth_to_cmap(head_depth_img)
                    cv2.imshow("head_depth_cmap", head_depth_cmap)

                head_rgb_hdr, head_rgb_img = client.get_rgb("head")
                if head_rgb_hdr is not None:
                    cv2.imshow("head_rgb_img", head_rgb_img)

                left_depth_hdr, left_depth_img = client.get_depth("left")
                if left_depth_hdr is not None:
                    left_depth_cmap = depth_to_cmap(left_depth_img)
                    cv2.imshow("left_depth_cmap", left_depth_cmap)

                left_rgb_hdr, left_rgb_img = client.get_rgb("left")
                if left_rgb_hdr is not None:
                    cv2.imshow("left_rgb_img", left_rgb_img)

                right_depth_hdr, right_depth_img = client.get_depth("right")
                if right_depth_hdr is not None:
                    right_depth_cmap = depth_to_cmap(right_depth_img)
                    cv2.imshow("right_depth_cmap", right_depth_cmap)

                right_rgb_hdr, right_rgb_img = client.get_rgb("right")
                if right_rgb_hdr is not None:
                    cv2.imshow("right_rgb_img", right_rgb_img)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

            rate.sleep()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
