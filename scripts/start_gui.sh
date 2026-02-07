#!/bin/bash
# FASTLIO2 ROS2 一键启动脚本
# 用法: ./start_gui.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# 检查Python3
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到python3"
    exit 1
fi

# 检查tkinter
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "错误: 未安装tkinter, 请运行: sudo apt install python3-tk"
    exit 1
fi

# 启动GUI
echo "启动FASTLIO2 ROS2控制面板..."
python3 "$SCRIPT_DIR/gui_launcher.py"
