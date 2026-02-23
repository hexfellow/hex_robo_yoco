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


def calc_tau_comp(
    q_cur: np.ndarray,
    dq_cur: np.ndarray,
    dyn_util: DynUtil,
    dofs: dict,
):
    q_arm = q_cur[:dofs["robot_arm"]]
    dq_arm = dq_cur[:dofs["robot_arm"]]

    _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(q_arm, dq_arm)
    tau_comp = np.zeros(dofs["sum"])
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

    # get cam state and intri
    cam_state = client.get_cam_state()
    has_cam = cam_state["use_rgb"] or cam_state["use_depth"]
    if has_cam:
        intri = client.get_intri()
        hex_log(HEX_LOG_LEVEL["info"], f"intri: {intri}")

    # get states, rgb and depth, and set cmds
    rate = HexRate(1000)
    while True:
        rate.sleep()

        robot_states_hdr, robot_states = client.get_states()
        if robot_states_hdr is not None:
            q_cur = robot_states[:, 0]
            dq_cur = robot_states[:, 1]
            tau_comp = calc_tau_comp(
                q_cur,
                dq_cur,
                dyn_util,
                dofs,
            )
            cmds = np.vstack(
                (q_cur, np.zeros(dofs["sum"]), tau_comp, mit_kp, mit_kd)).T
            client.set_cmds(cmds)


if __name__ == '__main__':
    main()
