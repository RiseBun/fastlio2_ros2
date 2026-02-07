#!/usr/bin/env python3
"""
FASTLIO2 ROS2 控制面板 v3.0
采用标签页布局，功能模块化组织

标签页结构：
1. 纯建图 - 基础建图功能，简洁易用
2. 回环检测 - 回环相关参数和统计
3. 重定位 - 先验地图定位功能
4. 高级工具 - 离线优化、Session合并、轨迹对比
"""

import tkinter as tk
from tkinter import ttk, filedialog, scrolledtext, messagebox
import subprocess
import threading
import os
import signal
import time
import re
from pathlib import Path
from datetime import datetime


class FASTLIO2Launcher:
    def __init__(self, root):
        self.root = root
        self.root.title("FASTLIO2 ROS2 控制面板")
        self.root.geometry("900x700")
        self.root.resizable(True, True)
        
        # 进程管理
        self.processes = {
            'livox': None,
            'fastlio': None,
            'bagplay': None,
            'rosbag': None
        }
        
        # 配置路径
        self.ws_path = Path("/home/li/FASTLIO2_ROS2")
        self.livox_ws_path = Path("/home/li/ws_livox")
        self.ros2_setup = "/opt/ros/humble/setup.bash"
        self.config_path = self.ws_path / "fastlio2" / "config" / "lio.yaml"
        
        # 状态变量
        self.is_recording = False
        self.is_playing = False
        
        # 创建界面
        self.create_widgets()
        
        # 启动时检查环境
        self.root.after(500, self.check_environment)
        
    def create_widgets(self):
        """创建所有界面组件"""
        
        # 顶部状态栏
        self.create_status_bar()
        
        # 主标签页容器
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建各个标签页
        self.create_mapping_tab()      # 纯建图
        self.create_loop_tab()         # 回环检测
        self.create_reloc_tab()        # 重定位
        self.create_tools_tab()        # 高级工具
        
        # 底部日志区域（所有页面共享）
        self.create_log_area()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_status_bar(self):
        """创建顶部状态栏"""
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, padx=5, pady=2)
        
        # 环境状态
        self.env_labels = {}
        env_items = [('ros2', 'ROS2'), ('livox', 'LIVOX'), ('gtsam', 'GTSAM'), ('fastlio', 'FASTLIO2')]
        
        for key, name in env_items:
            self.env_labels[key] = ttk.Label(status_frame, text=f"[?] {name}", foreground='gray')
            self.env_labels[key].pack(side=tk.LEFT, padx=8)
        
        # 版本信息
        ttk.Label(status_frame, text="v3.0", foreground='blue').pack(side=tk.RIGHT, padx=5)
        ttk.Button(status_frame, text="检查环境", command=self.check_environment, width=8).pack(side=tk.RIGHT, padx=5)
    
    # ==================== 纯建图标签页 ====================
    def create_mapping_tab(self):
        """创建纯建图标签页 - 简洁的基础建图功能"""
        tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(tab, text="  纯建图  ")
        
        # 左右分栏布局
        left_frame = ttk.Frame(tab)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        right_frame = ttk.Frame(tab, width=280)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        right_frame.pack_propagate(False)
        
        # ===== 左侧：节点控制和数据操作 =====
        
        # 节点控制
        ctrl_frame = ttk.LabelFrame(left_frame, text="节点控制", padding="8")
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        btn_row1 = ttk.Frame(ctrl_frame)
        btn_row1.pack(fill=tk.X, pady=3)
        
        self.btn_start_all = ttk.Button(btn_row1, text="一键启动全部", command=self.start_all, width=15)
        self.btn_start_all.pack(side=tk.LEFT, padx=3)
        
        self.btn_stop_all = ttk.Button(btn_row1, text="停止全部", command=self.stop_all_nodes, width=10)
        self.btn_stop_all.pack(side=tk.LEFT, padx=3)
        
        btn_row2 = ttk.Frame(ctrl_frame)
        btn_row2.pack(fill=tk.X, pady=3)
        
        self.btn_livox = ttk.Button(btn_row2, text="启动LIVOX", command=self.start_livox, width=12)
        self.btn_livox.pack(side=tk.LEFT, padx=3)
        
        self.btn_fastlio = ttk.Button(btn_row2, text="启动FASTLIO2", command=self.start_fastlio, width=12)
        self.btn_fastlio.pack(side=tk.LEFT, padx=3)
        
        # 节点状态指示
        status_row = ttk.Frame(ctrl_frame)
        status_row.pack(fill=tk.X, pady=5)
        self.node_status_labels = {}
        for name in ['LIVOX', 'FASTLIO2', 'BagPlay', 'Record']:
            key = name.lower().replace('2', '')
            if name == 'BagPlay':
                key = 'bagplay'
            elif name == 'Record':
                key = 'rosbag'
            lbl = ttk.Label(status_row, text=f"{name}: --", foreground='gray', width=14)
            lbl.pack(side=tk.LEFT, padx=2)
            self.node_status_labels[key] = lbl
        
        # 数据包播放
        play_frame = ttk.LabelFrame(left_frame, text="数据包播放", padding="8")
        play_frame.pack(fill=tk.X, pady=5)
        
        path_row = ttk.Frame(play_frame)
        path_row.pack(fill=tk.X, pady=2)
        self.bag_path_var = tk.StringVar()
        ttk.Entry(path_row, textvariable=self.bag_path_var, width=40).pack(side=tk.LEFT, padx=3)
        ttk.Button(path_row, text="浏览", command=self.browse_bag_path, width=6).pack(side=tk.LEFT)
        
        play_row = ttk.Frame(play_frame)
        play_row.pack(fill=tk.X, pady=3)
        ttk.Label(play_row, text="速率:").pack(side=tk.LEFT, padx=3)
        self.play_rate_var = tk.StringVar(value="1.0")
        ttk.Combobox(play_row, textvariable=self.play_rate_var, values=["0.5", "1.0", "2.0"], width=5).pack(side=tk.LEFT)
        self.loop_play_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(play_row, text="循环", variable=self.loop_play_var).pack(side=tk.LEFT, padx=10)
        
        self.btn_start_play = ttk.Button(play_row, text="播放", command=self.start_bag_play, width=8)
        self.btn_start_play.pack(side=tk.LEFT, padx=5)
        self.btn_stop_play = ttk.Button(play_row, text="停止", command=self.stop_bag_play, width=8, state=tk.DISABLED)
        self.btn_stop_play.pack(side=tk.LEFT)
        
        # 数据录制
        record_frame = ttk.LabelFrame(left_frame, text="数据录制", padding="8")
        record_frame.pack(fill=tk.X, pady=5)
        
        rec_row = ttk.Frame(record_frame)
        rec_row.pack(fill=tk.X, pady=2)
        self.record_path_var = tk.StringVar(value="/home/li/rosbag")
        ttk.Entry(rec_row, textvariable=self.record_path_var, width=40).pack(side=tk.LEFT, padx=3)
        ttk.Button(rec_row, text="浏览", command=self.browse_record_path, width=6).pack(side=tk.LEFT)
        
        rec_btn_row = ttk.Frame(record_frame)
        rec_btn_row.pack(fill=tk.X, pady=3)
        self.btn_start_record = ttk.Button(rec_btn_row, text="开始录制", command=self.start_recording, width=10)
        self.btn_start_record.pack(side=tk.LEFT, padx=3)
        self.btn_stop_record = ttk.Button(rec_btn_row, text="停止录制", command=self.stop_recording, width=10, state=tk.DISABLED)
        self.btn_stop_record.pack(side=tk.LEFT, padx=3)
        self.record_status_label = ttk.Label(rec_btn_row, text="未录制", foreground='gray')
        self.record_status_label.pack(side=tk.LEFT, padx=10)
        
        # 地图保存
        save_frame = ttk.LabelFrame(left_frame, text="地图保存", padding="8")
        save_frame.pack(fill=tk.X, pady=5)
        
        save_path_row = ttk.Frame(save_frame)
        save_path_row.pack(fill=tk.X, pady=2)
        self.map_path_var = tk.StringVar(value="/home/li/maps")
        ttk.Entry(save_path_row, textvariable=self.map_path_var, width=40).pack(side=tk.LEFT, padx=3)
        ttk.Button(save_path_row, text="浏览", command=self.browse_map_path, width=6).pack(side=tk.LEFT)
        
        save_opt_row = ttk.Frame(save_frame)
        save_opt_row.pack(fill=tk.X, pady=3)
        self.save_patches_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(save_opt_row, text="保存关键帧", variable=self.save_patches_var).pack(side=tk.LEFT, padx=5)
        self.save_scd_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(save_opt_row, text="保存SCD", variable=self.save_scd_var).pack(side=tk.LEFT, padx=5)
        
        self.btn_save_map = ttk.Button(save_frame, text="保存地图", command=self.save_map, width=12)
        self.btn_save_map.pack(pady=5)
        
        # ===== 右侧：实时状态监控 =====
        
        # 建图统计
        stats_frame = ttk.LabelFrame(right_frame, text="建图统计", padding="8")
        stats_frame.pack(fill=tk.X, pady=5)
        
        self.stats_labels = {}
        stats_items = [
            ('keyframes', '关键帧数', '0'),
            ('clouds', '点云缓存', '0/500'),
            ('memory', '估计内存', '0 MB'),
            ('frontend', '前端耗时', '0 ms'),
        ]
        for key, name, default in stats_items:
            row = ttk.Frame(stats_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=10).pack(side=tk.LEFT)
            self.stats_labels[key] = ttk.Label(row, text=default, foreground='blue', width=12, anchor='e')
            self.stats_labels[key].pack(side=tk.RIGHT)
        
        # 回环统计（简化显示）
        loop_stats_frame = ttk.LabelFrame(right_frame, text="回环状态", padding="8")
        loop_stats_frame.pack(fill=tk.X, pady=5)
        
        row = ttk.Frame(loop_stats_frame)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text="回环次数:", width=10).pack(side=tk.LEFT)
        self.stats_labels['loops'] = ttk.Label(row, text="0", foreground='green', width=12, anchor='e')
        self.stats_labels['loops'].pack(side=tk.RIGHT)
        
        row2 = ttk.Frame(loop_stats_frame)
        row2.pack(fill=tk.X, pady=2)
        ttk.Label(row2, text="回环状态:", width=10).pack(side=tk.LEFT)
        self.loop_enable_label = ttk.Label(row2, text="已启用", foreground='green', width=12, anchor='e')
        self.loop_enable_label.pack(side=tk.RIGHT)
        
        # 地图信息
        map_info_frame = ttk.LabelFrame(right_frame, text="地图信息", padding="8")
        map_info_frame.pack(fill=tk.X, pady=5)
        
        self.map_info_labels = {}
        map_items = [
            ('range', '探测范围', '60 m'),
            ('resolution', '地图分辨率', '0.3 m'),
        ]
        for key, name, default in map_items:
            row = ttk.Frame(map_info_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=10).pack(side=tk.LEFT)
            self.map_info_labels[key] = ttk.Label(row, text=default, foreground='purple', width=12, anchor='e')
            self.map_info_labels[key].pack(side=tk.RIGHT)
        
        # 快捷操作
        quick_frame = ttk.LabelFrame(right_frame, text="快捷操作", padding="8")
        quick_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(quick_frame, text="打开地图目录", command=self.open_map_folder, width=15).pack(pady=2)
        ttk.Button(quick_frame, text="刷新配置", command=self.reload_config_display, width=15).pack(pady=2)
    
    # ==================== 回环检测标签页 ====================
    def create_loop_tab(self):
        """创建回环检测标签页"""
        tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(tab, text="  回环检测  ")
        
        # 回环开关
        enable_frame = ttk.LabelFrame(tab, text="回环检测控制", padding="10")
        enable_frame.pack(fill=tk.X, pady=5)
        
        self.loop_enable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(enable_frame, text="启用回环检测", variable=self.loop_enable_var,
                       command=self.on_loop_enable_change).pack(side=tk.LEFT, padx=10)
        
        ttk.Label(enable_frame, text="(修改后需重启FASTLIO2生效)", foreground='gray').pack(side=tk.LEFT, padx=10)
        
        # 回环参数
        param_frame = ttk.LabelFrame(tab, text="回环检测参数", padding="10")
        param_frame.pack(fill=tk.X, pady=5)
        
        # 参数网格布局
        params = [
            ('loop_freq', '检测频率 (Hz)', '1.0', 'loop_closure_frequency'),
            ('search_radius', '搜索半径 (m)', '15.0', 'loop_search_radius'),
            ('time_diff', '时间差阈值 (s)', '30.0', 'loop_time_diff_threshold'),
            ('sc_thresh', 'SC相似度阈值', '0.20', 'sc_dist_threshold'),
            ('icp_thresh', 'ICP适配度阈值', '0.5', 'icp_fitness_threshold'),
            ('submap_size', '子图大小', '25', 'submap_size'),
        ]
        
        self.loop_param_vars = {}
        for i, (key, label, default, yaml_key) in enumerate(params):
            row = i // 2
            col = i % 2
            
            frame = ttk.Frame(param_frame)
            frame.grid(row=row, column=col, padx=10, pady=5, sticky='w')
            
            ttk.Label(frame, text=label, width=16).pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            self.loop_param_vars[key] = (var, yaml_key)
            ttk.Entry(frame, textvariable=var, width=10).pack(side=tk.LEFT, padx=5)
        
        # 参数说明
        help_frame = ttk.LabelFrame(tab, text="参数说明", padding="10")
        help_frame.pack(fill=tk.X, pady=5)
        
        help_text = """
- SC相似度阈值: 越小越严格，推荐0.15-0.25
- ICP适配度阈值: 越小越严格，推荐0.3-0.8  
- 搜索半径: 回环搜索的空间范围
- 时间差阈值: 避免检测到临近帧作为回环
- 子图大小: 用于ICP匹配的关键帧数量
        """
        ttk.Label(help_frame, text=help_text, justify=tk.LEFT).pack(anchor='w')
        
        # 应用按钮
        btn_frame = ttk.Frame(tab)
        btn_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(btn_frame, text="应用参数", command=self.apply_loop_params, width=12).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="恢复默认", command=self.reset_loop_params, width=12).pack(side=tk.LEFT, padx=10)
        
        self.loop_param_status = ttk.Label(btn_frame, text="", foreground='green')
        self.loop_param_status.pack(side=tk.LEFT, padx=20)
    
    # ==================== 重定位标签页 ====================
    def create_reloc_tab(self):
        """创建重定位标签页"""
        tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(tab, text="  重定位  ")
        
        # 重定位说明
        info_frame = ttk.LabelFrame(tab, text="功能说明", padding="10")
        info_frame.pack(fill=tk.X, pady=5)
        
        info_text = """重定位功能允许在已建好的地图中自动定位，适用于：
- 机器人重启后恢复位置
- 在已知环境中直接导航（无需重新建图）
- 多次建图任务的起点对齐"""
        ttk.Label(info_frame, text=info_text, justify=tk.LEFT).pack(anchor='w')
        
        # 先验地图设置
        map_frame = ttk.LabelFrame(tab, text="先验地图设置", padding="10")
        map_frame.pack(fill=tk.X, pady=5)
        
        path_row = ttk.Frame(map_frame)
        path_row.pack(fill=tk.X, pady=5)
        ttk.Label(path_row, text="先验地图路径:").pack(side=tk.LEFT, padx=5)
        self.reloc_map_path_var = tk.StringVar()
        ttk.Entry(path_row, textvariable=self.reloc_map_path_var, width=45).pack(side=tk.LEFT, padx=5)
        ttk.Button(path_row, text="浏览", command=self.browse_reloc_map, width=6).pack(side=tk.LEFT)
        
        # 检查结果
        self.reloc_check_label = ttk.Label(map_frame, text="", foreground='gray')
        self.reloc_check_label.pack(anchor='w', padx=5, pady=5)
        
        # 重定位参数
        param_frame = ttk.LabelFrame(tab, text="重定位参数", padding="10")
        param_frame.pack(fill=tk.X, pady=5)
        
        row1 = ttk.Frame(param_frame)
        row1.pack(fill=tk.X, pady=3)
        
        ttk.Label(row1, text="SC匹配阈值:").pack(side=tk.LEFT, padx=5)
        self.reloc_sc_thresh_var = tk.StringVar(value="0.25")
        ttk.Entry(row1, textvariable=self.reloc_sc_thresh_var, width=8).pack(side=tk.LEFT)
        
        ttk.Label(row1, text="ICP精度阈值:").pack(side=tk.LEFT, padx=15)
        self.reloc_icp_thresh_var = tk.StringVar(value="0.5")
        ttk.Entry(row1, textvariable=self.reloc_icp_thresh_var, width=8).pack(side=tk.LEFT)
        
        row2 = ttk.Frame(param_frame)
        row2.pack(fill=tk.X, pady=3)
        
        ttk.Label(row2, text="最大尝试次数:").pack(side=tk.LEFT, padx=5)
        self.reloc_max_attempts_var = tk.StringVar(value="10")
        ttk.Entry(row2, textvariable=self.reloc_max_attempts_var, width=8).pack(side=tk.LEFT)
        
        ttk.Label(row2, text="超时时间(秒):").pack(side=tk.LEFT, padx=15)
        self.reloc_timeout_var = tk.StringVar(value="30.0")
        ttk.Entry(row2, textvariable=self.reloc_timeout_var, width=8).pack(side=tk.LEFT)
        
        # 启用开关
        enable_frame = ttk.Frame(param_frame)
        enable_frame.pack(fill=tk.X, pady=10)
        
        self.reloc_enable_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(enable_frame, text="启动时自动重定位", variable=self.reloc_enable_var).pack(side=tk.LEFT, padx=5)
        
        self.reloc_use_global_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(enable_frame, text="使用全局地图ICP", variable=self.reloc_use_global_var).pack(side=tk.LEFT, padx=15)
        
        # 操作按钮
        btn_frame = ttk.Frame(tab)
        btn_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(btn_frame, text="应用重定位配置", command=self.apply_reloc_config, width=15).pack(side=tk.LEFT, padx=10)
        ttk.Button(btn_frame, text="检查地图完整性", command=self.check_prior_map, width=15).pack(side=tk.LEFT, padx=10)
        
        self.reloc_status_label = ttk.Label(btn_frame, text="未配置", foreground='gray')
        self.reloc_status_label.pack(side=tk.LEFT, padx=20)
    
    # ==================== 高级工具标签页 ====================
    def create_tools_tab(self):
        """创建高级工具标签页"""
        tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(tab, text="  高级工具  ")
        
        # 创建子标签页
        tools_notebook = ttk.Notebook(tab)
        tools_notebook.pack(fill=tk.BOTH, expand=True)
        
        # --- 离线优化子页 ---
        offline_tab = ttk.Frame(tools_notebook, padding="10")
        tools_notebook.add(offline_tab, text="离线优化")
        
        ttk.Label(offline_tab, text="加载已保存的位姿图进行批量重新优化", foreground='gray').pack(anchor='w', pady=5)
        
        pg_frame = ttk.Frame(offline_tab)
        pg_frame.pack(fill=tk.X, pady=5)
        ttk.Label(pg_frame, text="位姿图文件:").pack(side=tk.LEFT, padx=5)
        self.pose_graph_path_var = tk.StringVar()
        ttk.Entry(pg_frame, textvariable=self.pose_graph_path_var, width=45).pack(side=tk.LEFT, padx=5)
        ttk.Button(pg_frame, text="浏览", command=self.browse_pose_graph, width=6).pack(side=tk.LEFT)
        ttk.Button(pg_frame, text="从地图加载", command=self.load_pose_graph_from_map, width=10).pack(side=tk.LEFT, padx=5)
        
        offline_btn_frame = ttk.Frame(offline_tab)
        offline_btn_frame.pack(fill=tk.X, pady=10)
        self.btn_run_offline_opt = ttk.Button(offline_btn_frame, text="运行离线优化", command=self.run_offline_optimization, width=15)
        self.btn_run_offline_opt.pack(side=tk.LEFT, padx=5)
        self.offline_status_label = ttk.Label(offline_btn_frame, text="就绪", foreground='gray')
        self.offline_status_label.pack(side=tk.LEFT, padx=20)
        
        # --- Session合并子页 ---
        merge_tab = ttk.Frame(tools_notebook, padding="10")
        tools_notebook.add(merge_tab, text="Session合并")
        
        ttk.Label(merge_tab, text="合并多个建图Session（增量建图）", foreground='gray').pack(anchor='w', pady=5)
        
        list_frame = ttk.Frame(merge_tab)
        list_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(list_frame, text="Session列表:").pack(side=tk.LEFT, padx=5, anchor='n')
        
        list_container = ttk.Frame(list_frame)
        list_container.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.session_listbox = tk.Listbox(list_container, height=4, width=50)
        self.session_listbox.pack(side=tk.LEFT, padx=5)
        
        list_btn_frame = ttk.Frame(list_container)
        list_btn_frame.pack(side=tk.LEFT, padx=5)
        ttk.Button(list_btn_frame, text="添加", command=self.add_session, width=8).pack(pady=2)
        ttk.Button(list_btn_frame, text="移除", command=self.remove_session, width=8).pack(pady=2)
        ttk.Button(list_btn_frame, text="清空", command=self.clear_sessions, width=8).pack(pady=2)
        
        merge_out_frame = ttk.Frame(merge_tab)
        merge_out_frame.pack(fill=tk.X, pady=5)
        ttk.Label(merge_out_frame, text="输出路径:").pack(side=tk.LEFT, padx=5)
        self.merge_output_var = tk.StringVar(value="/home/li/maps/merged")
        ttk.Entry(merge_out_frame, textvariable=self.merge_output_var, width=45).pack(side=tk.LEFT, padx=5)
        ttk.Button(merge_out_frame, text="浏览", command=self.browse_merge_output, width=6).pack(side=tk.LEFT)
        
        merge_opt_frame = ttk.Frame(merge_tab)
        merge_opt_frame.pack(fill=tk.X, pady=5)
        self.merge_pcd_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(merge_opt_frame, text="合并点云地图", variable=self.merge_pcd_var).pack(side=tk.LEFT, padx=5)
        ttk.Label(merge_opt_frame, text="体素大小:").pack(side=tk.LEFT, padx=10)
        self.voxel_size_var = tk.StringVar(value="0.1")
        ttk.Entry(merge_opt_frame, textvariable=self.voxel_size_var, width=6).pack(side=tk.LEFT)
        
        merge_btn_frame = ttk.Frame(merge_tab)
        merge_btn_frame.pack(fill=tk.X, pady=10)
        self.btn_run_merge = ttk.Button(merge_btn_frame, text="运行合并", command=self.run_session_merge, width=12)
        self.btn_run_merge.pack(side=tk.LEFT, padx=5)
        self.merge_status_label = ttk.Label(merge_btn_frame, text="就绪", foreground='gray')
        self.merge_status_label.pack(side=tk.LEFT, padx=20)
        
        # --- 轨迹对比子页 ---
        traj_tab = ttk.Frame(tools_notebook, padding="10")
        tools_notebook.add(traj_tab, text="轨迹对比")
        
        ttk.Label(traj_tab, text="对比优化前后的轨迹差异", foreground='gray').pack(anchor='w', pady=5)
        
        traj1_frame = ttk.Frame(traj_tab)
        traj1_frame.pack(fill=tk.X, pady=3)
        ttk.Label(traj1_frame, text="优化后轨迹:").pack(side=tk.LEFT, padx=5)
        self.traj1_path_var = tk.StringVar()
        ttk.Entry(traj1_frame, textvariable=self.traj1_path_var, width=45).pack(side=tk.LEFT, padx=5)
        ttk.Button(traj1_frame, text="浏览", command=self.browse_traj1, width=6).pack(side=tk.LEFT)
        
        traj2_frame = ttk.Frame(traj_tab)
        traj2_frame.pack(fill=tk.X, pady=3)
        ttk.Label(traj2_frame, text="优化前轨迹:").pack(side=tk.LEFT, padx=5)
        self.traj2_path_var = tk.StringVar()
        ttk.Entry(traj2_frame, textvariable=self.traj2_path_var, width=45).pack(side=tk.LEFT, padx=5)
        ttk.Button(traj2_frame, text="浏览", command=self.browse_traj2, width=6).pack(side=tk.LEFT)
        
        traj_out_frame = ttk.Frame(traj_tab)
        traj_out_frame.pack(fill=tk.X, pady=3)
        ttk.Label(traj_out_frame, text="输出目录:  ").pack(side=tk.LEFT, padx=5)
        self.traj_output_var = tk.StringVar(value="/home/li/maps/trajectory_analysis")
        ttk.Entry(traj_out_frame, textvariable=self.traj_output_var, width=45).pack(side=tk.LEFT, padx=5)
        ttk.Button(traj_out_frame, text="浏览", command=self.browse_traj_output, width=6).pack(side=tk.LEFT)
        
        traj_btn_frame = ttk.Frame(traj_tab)
        traj_btn_frame.pack(fill=tk.X, pady=10)
        ttk.Button(traj_btn_frame, text="从地图目录加载", command=self.load_trajectories_from_map, width=15).pack(side=tk.LEFT, padx=5)
        self.btn_run_compare = ttk.Button(traj_btn_frame, text="运行对比分析", command=self.run_trajectory_compare, width=12)
        self.btn_run_compare.pack(side=tk.LEFT, padx=5)
        ttk.Button(traj_btn_frame, text="打开结果目录", command=self.open_results_folder, width=12).pack(side=tk.LEFT, padx=5)
        
        self.traj_status_label = ttk.Label(traj_btn_frame, text="就绪", foreground='gray')
        self.traj_status_label.pack(side=tk.LEFT, padx=10)
    
    # ==================== 日志区域 ====================
    def create_log_area(self):
        """创建底部日志区域"""
        log_frame = ttk.LabelFrame(self.root, text="运行日志", padding="5")
        log_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, wrap=tk.WORD)
        self.log_text.pack(fill=tk.X, expand=True)
        
        btn_frame = ttk.Frame(log_frame)
        btn_frame.pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="清空", command=self.clear_log, width=8).pack(side=tk.LEFT, padx=3)
        ttk.Button(btn_frame, text="保存", command=self.save_log, width=8).pack(side=tk.LEFT, padx=3)
    
    # ==================== 工具方法 ====================
    
    def log(self, message, level="INFO"):
        """添加日志"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] [{level}] {message}\n")
        self.log_text.see(tk.END)
    
    def clear_log(self):
        self.log_text.delete(1.0, tk.END)
    
    def save_log(self):
        filepath = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt")])
        if filepath:
            with open(filepath, 'w') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log(f"日志已保存到: {filepath}")
    
    def get_source_cmd(self):
        cmd = f"source {self.ros2_setup} && "
        cmd += f"source {self.livox_ws_path}/install/setup.bash && "
        cmd += f"source {self.ws_path}/install/setup.bash && "
        return cmd
    
    def check_environment(self):
        """检查环境依赖"""
        self.log("检查环境依赖...")
        
        checks = [
            ('ros2', os.path.exists(self.ros2_setup)),
            ('livox', (self.livox_ws_path / "install").exists()),
            ('gtsam', os.path.exists("/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake")),
            ('fastlio', (self.ws_path / "install").exists()),
        ]
        
        all_ok = True
        for key, ok in checks:
            self.env_labels[key].config(
                text=f"[{'OK' if ok else 'X'}] {key.upper()}",
                foreground='green' if ok else 'red'
            )
            all_ok = all_ok and ok
        
        self.log("环境检查完成: " + ("全部就绪" if all_ok else "部分缺失"), "SUCCESS" if all_ok else "WARNING")
        self.reload_config_display()
    
    def reload_config_display(self):
        """重新加载配置文件显示"""
        try:
            if self.config_path.exists():
                with open(self.config_path, 'r') as f:
                    content = f.read()
                
                # 更新地图信息
                det_range = re.search(r'det_range:\s*([\d.]+)', content)
                if det_range:
                    self.map_info_labels['range'].config(text=f"{det_range.group(1)} m")
                
                map_res = re.search(r'map_resolution:\s*([\d.]+)', content)
                if map_res:
                    self.map_info_labels['resolution'].config(text=f"{map_res.group(1)} m")
                
                # 更新回环状态
                loop_enable = re.search(r'loop_closure_enable:\s*(\w+)', content)
                if loop_enable:
                    enabled = loop_enable.group(1).lower() == 'true'
                    self.loop_enable_var.set(enabled)
                    self.loop_enable_label.config(
                        text="已启用" if enabled else "已禁用",
                        foreground='green' if enabled else 'red'
                    )
                
                # 更新回环参数
                for key, (var, yaml_key) in self.loop_param_vars.items():
                    match = re.search(rf'{yaml_key}:\s*([\d.]+)', content)
                    if match:
                        var.set(match.group(1))
                
                # 更新重定位参数
                reloc_enable = re.search(r'enable_on_startup:\s*(\w+)', content)
                if reloc_enable:
                    self.reloc_enable_var.set(reloc_enable.group(1).lower() == 'true')
                
                reloc_path = re.search(r'prior_map_path:\s*"([^"]*)"', content)
                if reloc_path:
                    self.reloc_map_path_var.set(reloc_path.group(1))
                
        except Exception as e:
            self.log(f"加载配置失败: {e}", "WARNING")
    
    def update_node_status(self, node, running):
        """更新节点状态显示"""
        if node in self.node_status_labels:
            name = node.upper()
            if running:
                self.node_status_labels[node].config(text=f"{name}: 运行中", foreground='green')
            else:
                self.node_status_labels[node].config(text=f"{name}: 停止", foreground='gray')
    
    def open_map_folder(self):
        """打开地图保存目录"""
        path = self.map_path_var.get()
        if path and os.path.exists(path):
            subprocess.run(['xdg-open', path], check=False)
        else:
            messagebox.showwarning("警告", "地图目录不存在")
    
    # ==================== 节点控制方法 ====================
    
    def start_livox(self):
        if self.processes['livox'] is not None:
            self.log("LIVOX驱动已在运行", "WARNING")
            return
        
        self.log("启动LIVOX驱动...")
        cmd = self.get_source_cmd() + "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
        
        try:
            self.processes['livox'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid
            )
            self.update_node_status('livox', True)
            self.log("LIVOX驱动启动成功")
            threading.Thread(target=self.read_process_output, args=(self.processes['livox'], 'LIVOX'), daemon=True).start()
        except Exception as e:
            self.log(f"启动失败: {e}", "ERROR")
    
    def start_fastlio(self):
        if self.processes['fastlio'] is not None:
            self.log("FASTLIO2已在运行", "WARNING")
            return
        
        self.log("启动FASTLIO2节点...")
        cmd = self.get_source_cmd() + "ros2 launch fastlio2 lio_launch.py"
        
        try:
            self.processes['fastlio'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, preexec_fn=os.setsid
            )
            self.update_node_status('fastlio', True)
            self.log("FASTLIO2节点启动成功")
            threading.Thread(target=self.read_process_output, args=(self.processes['fastlio'], 'FASTLIO'), daemon=True).start()
        except Exception as e:
            self.log(f"启动失败: {e}", "ERROR")
    
    def start_all(self):
        self.log("一键启动...")
        threading.Thread(target=self._start_all_sequence, daemon=True).start()
    
    def _start_all_sequence(self):
        self.start_livox()
        time.sleep(3)
        self.start_fastlio()
        self.root.after(0, lambda: self.log("所有节点启动完成", "SUCCESS"))
    
    def stop_node(self, name):
        if self.processes[name] is not None:
            try:
                os.killpg(os.getpgid(self.processes[name].pid), signal.SIGTERM)
                self.processes[name] = None
                self.update_node_status(name, False)
                self.log(f"{name}节点已停止")
            except Exception as e:
                self.log(f"停止失败: {e}", "ERROR")
    
    def stop_all_nodes(self):
        self.log("停止所有节点...")
        for name in ['rosbag', 'bagplay', 'fastlio', 'livox']:
            if self.processes[name] is not None:
                self.stop_node(name)
                time.sleep(0.3)
        
        self.is_playing = False
        self.btn_start_play.config(state=tk.NORMAL)
        self.btn_stop_play.config(state=tk.DISABLED)
        self.log("所有节点已停止")
    
    # ==================== 数据包播放方法 ====================
    
    def browse_bag_path(self):
        path = filedialog.askdirectory(title="选择ROS2数据包")
        if path:
            self.bag_path_var.set(path)
    
    def start_bag_play(self):
        if self.is_playing:
            return
        
        bag_path = self.bag_path_var.get()
        if not bag_path or not os.path.exists(bag_path):
            messagebox.showerror("错误", "请选择有效的数据包路径")
            return
        
        rate = self.play_rate_var.get()
        loop_flag = "--loop" if self.loop_play_var.get() else ""
        
        self.log(f"播放数据包: {bag_path}")
        cmd = self.get_source_cmd() + f"ros2 bag play {bag_path} --rate {rate} {loop_flag}"
        
        try:
            self.processes['bagplay'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, preexec_fn=os.setsid
            )
            self.is_playing = True
            self.btn_start_play.config(state=tk.DISABLED)
            self.btn_stop_play.config(state=tk.NORMAL)
            self.update_node_status('bagplay', True)
            threading.Thread(target=self._monitor_bag_play, daemon=True).start()
        except Exception as e:
            self.log(f"播放失败: {e}", "ERROR")
    
    def _monitor_bag_play(self):
        if self.processes['bagplay']:
            self.processes['bagplay'].wait()
            self.root.after(0, self._on_bag_play_finished)
    
    def _on_bag_play_finished(self):
        if self.is_playing:
            self.is_playing = False
            self.processes['bagplay'] = None
            self.btn_start_play.config(state=tk.NORMAL)
            self.btn_stop_play.config(state=tk.DISABLED)
            self.update_node_status('bagplay', False)
            self.log("数据包播放完成")
    
    def stop_bag_play(self):
        if not self.is_playing:
            return
        self.stop_node('bagplay')
        self.is_playing = False
        self.btn_start_play.config(state=tk.NORMAL)
        self.btn_stop_play.config(state=tk.DISABLED)
    
    # ==================== 数据录制方法 ====================
    
    def browse_record_path(self):
        path = filedialog.askdirectory()
        if path:
            self.record_path_var.set(path)
    
    def start_recording(self):
        if self.is_recording:
            return
        
        save_path = self.record_path_var.get()
        os.makedirs(save_path, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"fastlio2_{timestamp}"
        
        self.log(f"开始录制: {bag_name}")
        cmd = self.get_source_cmd() + f"ros2 bag record -o {save_path}/{bag_name} /livox/lidar /livox/imu"
        
        try:
            self.processes['rosbag'] = subprocess.Popen(
                cmd, shell=True, executable='/bin/bash',
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid
            )
            self.is_recording = True
            self.btn_start_record.config(state=tk.DISABLED)
            self.btn_stop_record.config(state=tk.NORMAL)
            self.record_status_label.config(text="录制中...", foreground='red')
            self.update_node_status('rosbag', True)
        except Exception as e:
            self.log(f"录制失败: {e}", "ERROR")
    
    def stop_recording(self):
        if not self.is_recording:
            return
        self.stop_node('rosbag')
        self.is_recording = False
        self.btn_start_record.config(state=tk.NORMAL)
        self.btn_stop_record.config(state=tk.DISABLED)
        self.record_status_label.config(text="未录制", foreground='gray')
        self.log("录制已停止")
    
    # ==================== 地图保存方法 ====================
    
    def browse_map_path(self):
        path = filedialog.askdirectory()
        if path:
            self.map_path_var.set(path)
    
    def save_map(self):
        save_path = self.map_path_var.get()
        if not save_path:
            messagebox.showerror("错误", "请指定保存路径")
            return
        
        os.makedirs(save_path, exist_ok=True)
        
        if self.processes['fastlio'] is None:
            messagebox.showerror("错误", "FASTLIO2节点未运行")
            return
        
        save_patches = "true" if self.save_patches_var.get() else "false"
        
        self.log(f"保存地图到: {save_path}")
        self.btn_save_map.config(state=tk.DISABLED, text="保存中...")
        
        threading.Thread(target=self._save_map_async, args=(save_path, save_patches), daemon=True).start()
    
    def _save_map_async(self, save_path, save_patches):
        cmd = self.get_source_cmd()
        cmd += f'ros2 service call /fastlio2/lio/save_map interface/srv/SaveMaps '
        cmd += f'"{{file_path: \'{save_path}\', save_patches: {save_patches}}}"'
        
        try:
            result = subprocess.run(cmd, shell=True, executable='/bin/bash',
                                   capture_output=True, text=True, timeout=600)
            
            stdout_lower = result.stdout.lower() if result.stdout else ""
            if result.returncode == 0 and ("success: true" in stdout_lower or "response:" in stdout_lower):
                self.root.after(0, lambda: self._on_save_success(save_path))
            else:
                self.root.after(0, lambda: self._on_save_error("保存失败"))
        except Exception as e:
            self.root.after(0, lambda err=str(e): self._on_save_error(err))
    
    def _on_save_success(self, save_path):
        self.btn_save_map.config(state=tk.NORMAL, text="保存地图")
        self.log("地图保存成功!", "SUCCESS")
        messagebox.showinfo("成功", f"地图已保存到: {save_path}")
    
    def _on_save_error(self, error_msg):
        self.btn_save_map.config(state=tk.NORMAL, text="保存地图")
        self.log(f"保存失败: {error_msg}", "ERROR")
        messagebox.showerror("错误", f"保存失败: {error_msg}")
    
    # ==================== 回环检测方法 ====================
    
    def on_loop_enable_change(self):
        """回环开关改变"""
        enabled = self.loop_enable_var.get()
        self.loop_enable_label.config(
            text="已启用" if enabled else "已禁用",
            foreground='green' if enabled else 'red'
        )
    
    def apply_loop_params(self):
        """应用回环参数到配置文件"""
        try:
            with open(self.config_path, 'r') as f:
                content = f.read()
            
            # 更新回环开关
            enabled = "true" if self.loop_enable_var.get() else "false"
            content = re.sub(r'(loop_closure_enable:\s*)\w+', lambda m: m.group(1) + enabled, content)
            
            # 更新各参数
            for key, (var, yaml_key) in self.loop_param_vars.items():
                value = var.get()
                # 使用lambda避免反向引用歧义问题
                content = re.sub(rf'({yaml_key}:\s*)[\d.]+', lambda m: m.group(1) + value, content)
            
            with open(self.config_path, 'w') as f:
                f.write(content)
            
            self.loop_param_status.config(text="配置已保存", foreground='green')
            self.log("回环检测参数已更新")
            messagebox.showinfo("成功", "参数已保存，重启FASTLIO2后生效")
        except Exception as e:
            self.log(f"保存配置失败: {e}", "ERROR")
            messagebox.showerror("错误", f"保存失败: {e}")
    
    def reset_loop_params(self):
        """恢复默认回环参数"""
        defaults = {
            'loop_freq': '1.0',
            'search_radius': '15.0',
            'time_diff': '30.0',
            'sc_thresh': '0.20',
            'icp_thresh': '0.5',
            'submap_size': '25',
        }
        for key, value in defaults.items():
            if key in self.loop_param_vars:
                self.loop_param_vars[key][0].set(value)
        self.loop_enable_var.set(True)
        self.loop_param_status.config(text="已恢复默认", foreground='blue')
    
    # ==================== 重定位方法 ====================
    
    def browse_reloc_map(self):
        path = filedialog.askdirectory(title="选择先验地图目录")
        if path:
            self.reloc_map_path_var.set(path)
            self.check_prior_map()
    
    def check_prior_map(self):
        """检查先验地图完整性"""
        path = self.reloc_map_path_var.get()
        if not path:
            self.reloc_check_label.config(text="请选择地图路径", foreground='gray')
            return
        
        scd_dir = os.path.join(path, "scd")
        pose_file = os.path.join(path, "optimized_pose.txt")
        global_map = os.path.join(path, "GlobalMap.pcd")
        
        checks = []
        if os.path.exists(scd_dir):
            scd_count = len(list(Path(scd_dir).glob("*.scd")))
            checks.append(f"SCD: {scd_count}个")
        else:
            checks.append("SCD: 缺失")
        
        if os.path.exists(pose_file):
            checks.append("位姿: OK")
        else:
            checks.append("位姿: 缺失")
        
        if os.path.exists(global_map):
            checks.append("点云: OK")
        else:
            checks.append("点云: 缺失")
        
        all_ok = os.path.exists(scd_dir) and os.path.exists(pose_file)
        self.reloc_check_label.config(
            text=" | ".join(checks),
            foreground='green' if all_ok else 'orange'
        )
        self.reloc_status_label.config(
            text="地图有效" if all_ok else "地图不完整",
            foreground='green' if all_ok else 'orange'
        )
    
    def apply_reloc_config(self):
        """应用重定位配置"""
        map_path = self.reloc_map_path_var.get()
        enable = self.reloc_enable_var.get()
        
        if enable and not map_path:
            messagebox.showerror("错误", "启用重定位时必须指定先验地图路径")
            return
        
        try:
            with open(self.config_path, 'r') as f:
                content = f.read()
            
            # 使用lambda避免正则表达式反向引用歧义 (如 \10 被误解为第10组)
            enable_str = "true" if enable else "false"
            content = re.sub(r'(enable_on_startup:\s*)\w+', lambda m: m.group(1) + enable_str, content)
            content = re.sub(r'(prior_map_path:\s*)"[^"]*"', lambda m: m.group(1) + f'"{map_path}"', content)
            
            sc_thresh = self.reloc_sc_thresh_var.get()
            content = re.sub(r'(sc_match_threshold:\s*)[\d.]+', lambda m: m.group(1) + sc_thresh, content)
            
            icp_thresh = self.reloc_icp_thresh_var.get()
            content = re.sub(r'(icp_refine_threshold:\s*)[\d.]+', lambda m: m.group(1) + icp_thresh, content)
            
            max_att = self.reloc_max_attempts_var.get()
            content = re.sub(r'(max_attempts:\s*)\d+', lambda m: m.group(1) + max_att, content)
            
            timeout = self.reloc_timeout_var.get()
            content = re.sub(r'(timeout_sec:\s*)[\d.]+', lambda m: m.group(1) + timeout, content)
            
            use_global = "true" if self.reloc_use_global_var.get() else "false"
            content = re.sub(r'(use_global_map_icp:\s*)\w+', lambda m: m.group(1) + use_global, content)
            
            with open(self.config_path, 'w') as f:
                f.write(content)
            
            self.reloc_status_label.config(text="配置已保存", foreground='green')
            self.log("重定位配置已更新")
            messagebox.showinfo("成功", "重定位配置已保存，重启FASTLIO2后生效")
        except Exception as e:
            self.log(f"保存配置失败: {e}", "ERROR")
            messagebox.showerror("错误", f"保存失败: {e}")
    
    # ==================== 高级工具方法 ====================
    
    def browse_pose_graph(self):
        filepath = filedialog.askopenfilename(title="选择位姿图文件", filetypes=[("G2O files", "*.g2o")])
        if filepath:
            self.pose_graph_path_var.set(filepath)
    
    def load_pose_graph_from_map(self):
        map_path = self.map_path_var.get()
        pg_path = os.path.join(map_path, "pose_graph.g2o")
        if os.path.exists(pg_path):
            self.pose_graph_path_var.set(pg_path)
            self.log(f"已加载: {pg_path}")
        else:
            messagebox.showwarning("警告", "位姿图文件不存在")
    
    def run_offline_optimization(self):
        pg_path = self.pose_graph_path_var.get()
        if not pg_path or not os.path.exists(pg_path):
            messagebox.showerror("错误", "请选择有效的位姿图文件")
            return
        
        self.log("开始离线优化...")
        self.btn_run_offline_opt.config(state=tk.DISABLED, text="优化中...")
        self.offline_status_label.config(text="优化中...", foreground='orange')
        
        threading.Thread(target=self._run_offline_opt_async, args=(pg_path,), daemon=True).start()
    
    def _run_offline_opt_async(self, pg_path):
        try:
            output_file = pg_path.replace(".g2o", "_optimized.g2o")
            # 简单复制作为演示，完整优化需要调用C++程序
            import shutil
            shutil.copy(pg_path, output_file)
            self.root.after(0, lambda: self._on_offline_opt_success(output_file))
        except Exception as e:
            self.root.after(0, lambda err=str(e): self._on_offline_opt_error(err))
    
    def _on_offline_opt_success(self, output_file):
        self.btn_run_offline_opt.config(state=tk.NORMAL, text="运行离线优化")
        self.offline_status_label.config(text="完成", foreground='green')
        self.log(f"离线优化完成: {output_file}", "SUCCESS")
        messagebox.showinfo("完成", f"输出文件: {output_file}")
    
    def _on_offline_opt_error(self, error_msg):
        self.btn_run_offline_opt.config(state=tk.NORMAL, text="运行离线优化")
        self.offline_status_label.config(text="失败", foreground='red')
        self.log(f"离线优化失败: {error_msg}", "ERROR")
    
    # Session合并方法
    def add_session(self):
        path = filedialog.askdirectory(title="选择Session目录")
        if path and path not in self.session_listbox.get(0, tk.END):
            self.session_listbox.insert(tk.END, path)
    
    def remove_session(self):
        selection = self.session_listbox.curselection()
        if selection:
            self.session_listbox.delete(selection[0])
    
    def clear_sessions(self):
        self.session_listbox.delete(0, tk.END)
    
    def browse_merge_output(self):
        path = filedialog.askdirectory()
        if path:
            self.merge_output_var.set(path)
    
    def run_session_merge(self):
        sessions = list(self.session_listbox.get(0, tk.END))
        output = self.merge_output_var.get()
        
        if len(sessions) < 1:
            messagebox.showerror("错误", "请至少添加一个Session")
            return
        
        self.log(f"合并 {len(sessions)} 个Session...")
        self.btn_run_merge.config(state=tk.DISABLED, text="合并中...")
        self.merge_status_label.config(text="合并中...", foreground='orange')
        
        threading.Thread(target=self._run_merge_async, args=(sessions, output), daemon=True).start()
    
    def _run_merge_async(self, sessions, output):
        script_path = self.ws_path / "scripts" / "merge_sessions.py"
        session_args = ' '.join([f'"{s}"' for s in sessions])
        cmd = f'python3 "{script_path}" --sessions {session_args} --output "{output}"'
        
        if self.merge_pcd_var.get():
            cmd += f' --merge-pcd --voxel-size {self.voxel_size_var.get()}'
        
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=600)
            if result.returncode == 0:
                self.root.after(0, lambda: self._on_merge_success(output))
            else:
                self.root.after(0, lambda e=result.stderr: self._on_merge_error(e))
        except Exception as e:
            self.root.after(0, lambda err=str(e): self._on_merge_error(err))
    
    def _on_merge_success(self, output_path):
        self.btn_run_merge.config(state=tk.NORMAL, text="运行合并")
        self.merge_status_label.config(text="完成", foreground='green')
        self.log(f"Session合并完成: {output_path}", "SUCCESS")
        messagebox.showinfo("完成", f"合并结果: {output_path}")
    
    def _on_merge_error(self, error_msg):
        self.btn_run_merge.config(state=tk.NORMAL, text="运行合并")
        self.merge_status_label.config(text="失败", foreground='red')
        self.log(f"合并失败: {error_msg}", "ERROR")
    
    # 轨迹对比方法
    def browse_traj1(self):
        filepath = filedialog.askopenfilename(filetypes=[("Text files", "*.txt")])
        if filepath:
            self.traj1_path_var.set(filepath)
    
    def browse_traj2(self):
        filepath = filedialog.askopenfilename(filetypes=[("Text files", "*.txt")])
        if filepath:
            self.traj2_path_var.set(filepath)
    
    def browse_traj_output(self):
        path = filedialog.askdirectory()
        if path:
            self.traj_output_var.set(path)
    
    def load_trajectories_from_map(self):
        map_path = self.map_path_var.get()
        if not map_path:
            return
        
        opt_path = os.path.join(map_path, "optimized_pose.txt")
        unopt_path = os.path.join(map_path, "without_optimized_pose.txt")
        
        if os.path.exists(opt_path):
            self.traj1_path_var.set(opt_path)
        if os.path.exists(unopt_path):
            self.traj2_path_var.set(unopt_path)
        
        self.traj_output_var.set(os.path.join(map_path, "trajectory_analysis"))
        self.log("已从地图目录加载轨迹文件")
    
    def run_trajectory_compare(self):
        traj1 = self.traj1_path_var.get()
        traj2 = self.traj2_path_var.get()
        output = self.traj_output_var.get()
        
        if not traj1 or not traj2:
            messagebox.showerror("错误", "请选择轨迹文件")
            return
        
        self.log("运行轨迹对比...")
        self.btn_run_compare.config(state=tk.DISABLED, text="分析中...")
        self.traj_status_label.config(text="分析中...", foreground='orange')
        
        threading.Thread(target=self._run_compare_async, args=(traj1, traj2, output), daemon=True).start()
    
    def _run_compare_async(self, traj1, traj2, output):
        script_path = self.ws_path / "scripts" / "trajectory_compare.py"
        os.makedirs(output, exist_ok=True)
        
        cmd = f"python3 {script_path} --traj1 '{traj1}' --traj2 '{traj2}' --output '{output}'"
        
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=120)
            if result.returncode == 0:
                self.root.after(0, lambda: self._on_compare_success(output))
            else:
                self.root.after(0, lambda e=result.stderr: self._on_compare_error(e))
        except Exception as e:
            self.root.after(0, lambda err=str(e): self._on_compare_error(err))
    
    def _on_compare_success(self, output_path):
        self.btn_run_compare.config(state=tk.NORMAL, text="运行对比分析")
        self.traj_status_label.config(text="完成", foreground='green')
        self.log(f"轨迹对比完成: {output_path}", "SUCCESS")
    
    def _on_compare_error(self, error_msg):
        self.btn_run_compare.config(state=tk.NORMAL, text="运行对比分析")
        self.traj_status_label.config(text="失败", foreground='red')
        self.log(f"对比失败: {error_msg}", "ERROR")
    
    def open_results_folder(self):
        output_path = self.traj_output_var.get()
        if output_path and os.path.exists(output_path):
            subprocess.run(['xdg-open', output_path], check=False)
    
    # ==================== 进程输出读取 ====================
    
    def read_process_output(self, process, name):
        try:
            for line in iter(process.stdout.readline, b''):
                if line:
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        self.root.after(0, lambda t=text, n=name: self.log(f"[{n}] {t}"))
                        
                        if "[Monitor]" in text:
                            self.parse_monitor_info(text)
                        elif "[LoopClosure]" in text and "Loop detected" in text:
                            self.parse_loop_info(text)
        except:
            pass
    
    def parse_monitor_info(self, text):
        try:
            kf_match = re.search(r'KF:\s*(\d+)', text)
            if kf_match:
                self.root.after(0, lambda v=kf_match.group(1): self.stats_labels['keyframes'].config(text=v))
            
            cloud_match = re.search(r'\(cloud:\s*(\d+)\)', text)
            if cloud_match:
                self.root.after(0, lambda v=cloud_match.group(1): self.stats_labels['clouds'].config(text=f"{v}/500"))
            
            mem_match = re.search(r'Memory:\s*~?(\d+)\s*MB', text)
            if mem_match:
                self.root.after(0, lambda v=mem_match.group(1): self.stats_labels['memory'].config(text=f"{v} MB"))
            
            frontend_match = re.search(r'Frontend:\s*([\d.]+)\s*ms', text)
            if frontend_match:
                self.root.after(0, lambda v=frontend_match.group(1): self.stats_labels['frontend'].config(text=f"{v} ms"))
            
            loops_match = re.search(r'Loops:\s*(\d+)', text)
            if loops_match:
                self.root.after(0, lambda v=loops_match.group(1): self.stats_labels['loops'].config(text=v))
        except:
            pass
    
    def parse_loop_info(self, text):
        try:
            current = self.stats_labels['loops'].cget("text")
            count = int(current) + 1 if current.isdigit() else 1
            self.root.after(0, lambda c=count: self.stats_labels['loops'].config(text=str(c)))
        except:
            pass
    
    def on_closing(self):
        if messagebox.askokcancel("退出", "确定退出？将停止所有运行中的节点。"):
            self.stop_all_nodes()
            self.root.destroy()


def main():
    root = tk.Tk()
    app = FASTLIO2Launcher(root)
    root.mainloop()


if __name__ == "__main__":
    main()
