#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time
from hex_robo_yoco import HexYocoArcherY6

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


def calc_tau_comp(
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

    # wait for yoco client to work
    if not wait_client_working(client):
        hex_log(HEX_LOG_LEVEL["err"], "yoco client is not working")
        return

    # get dofs and limits
    dofs = client.get_dofs()
    limits = client.get_limits()
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")
    hex_log(HEX_LOG_LEVEL["info"], f"limits: {limits}")

    # get cam state and intri
    cam_state = client.get_cam_state()
    has_cam = cam_state["use_rgb"] or cam_state["use_depth"]
    if has_cam:
        intri = client.get_intri()
        hex_log(HEX_LOG_LEVEL["info"], f"intri: {intri}")

    # get states, rgb and depth, and set cmds
    rate = HexRate(1000)
    while True:
        robot_states_hdr, robot_states = client.get_states()
        if robot_states_hdr is not None:
            q_cur = robot_states[:, 0]
            dq_cur = robot_states[:, 1]
            tau_comp = calc_tau_comp(
                q_cur,
                dq_cur,
                dyn_util,
                dofs,
                use_gripper,
            )
            cmds = np.vstack(
                (q_cur, np.zeros(dofs), tau_comp, mit_kp, mit_kd)).T
            client.set_cmds(cmds)
            rate.sleep()


if __name__ == '__main__':
    main()
