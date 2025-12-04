#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-11-20
################################################################

from .hex_yoco_e3_desktop import HexYocoE3Desktop
from .hex_yoco_archer_y6 import HexYocoArcherY6

import os

file_dir = os.path.dirname(os.path.abspath(__file__))
HEX_YOCO_DRIVER_PATH_DICT = {
    "archer_y6": f"{file_dir}/drivers/archer_y6_driver.py",
    "e3_desktop": f"{file_dir}/drivers/e3_desktop_driver.py",
}

__all__ = [
    "HexYocoE3Desktop",
    "HexYocoArcherY6",
    "HEX_YOCO_DRIVER_PATH_DICT",
]
