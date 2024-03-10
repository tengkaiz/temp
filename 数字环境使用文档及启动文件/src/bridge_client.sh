#!/bin/bash
PYTHON="/usr/bin/python3"  # 指定要使用的 Python 解释器路径
sudo $PYTHON ./bridge_client.py &
sleep 2
foxglove-studio >/dev/null 2>&1