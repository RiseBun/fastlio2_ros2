#!/usr/bin/env python3
"""
FASTLIO2 ROS2 轨迹对比分析工具

功能:
- 读取KITTI格式轨迹文件（优化前/优化后）
- 计算轨迹差异指标（APE、RPE、ATE等）
- 生成可视化对比图
- 输出详细差异报告

用法:
    python3 trajectory_compare.py --traj1 path/to/optimized_pose.txt \
                                  --traj2 path/to/without_optimized_pose.txt \
                                  --output path/to/output_folder
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，避免版本冲突
import matplotlib.pyplot as plt
import argparse
import os
from pathlib import Path
from datetime import datetime
import json


def load_kitti_trajectory(filepath):
    """
    加载KITTI格式轨迹文件
    每行12个数字，表示3x4变换矩阵 [R|t]
    """
    poses = []
    positions = []
    rotations = []
    
    with open(filepath, 'r') as f:
        for line_num, line in enumerate(f):
            values = line.strip().split()
            if len(values) != 12:
                print(f"Warning: Line {line_num} has {len(values)} values (expected 12)")
                continue
            
            values = [float(v) for v in values]
            
            # 构建4x4变换矩阵
            T = np.eye(4)
            T[0, :4] = values[0:4]
            T[1, :4] = values[4:8]
            T[2, :4] = values[8:12]
            
            poses.append(T)
            positions.append(T[:3, 3])
            rotations.append(T[:3, :3])
    
    return {
        'poses': np.array(poses),
        'positions': np.array(positions),
        'rotations': np.array(rotations),
        'num_poses': len(poses)
    }


def compute_ate(traj1, traj2):
    """
    计算绝对轨迹误差 (Absolute Trajectory Error)
    ATE = sqrt(1/n * sum(||p1_i - p2_i||^2))
    """
    n = min(len(traj1['positions']), len(traj2['positions']))
    if n == 0:
        return {'ate': 0, 'errors': [], 'stats': {}}
    
    errors = []
    for i in range(n):
        diff = traj1['positions'][i] - traj2['positions'][i]
        error = np.linalg.norm(diff)
        errors.append(error)
    
    errors = np.array(errors)
    ate = np.sqrt(np.mean(errors**2))
    
    return {
        'ate': ate,
        'rmse': ate,
        'mean': np.mean(errors),
        'std': np.std(errors),
        'max': np.max(errors),
        'min': np.min(errors),
        'median': np.median(errors),
        'errors': errors
    }


def compute_rpe(traj1, traj2, delta=1):
    """
    计算相对位姿误差 (Relative Pose Error)
    衡量局部一致性
    """
    n = min(len(traj1['poses']), len(traj2['poses']))
    if n <= delta:
        return {'rpe_trans': 0, 'rpe_rot': 0, 'errors': []}
    
    trans_errors = []
    rot_errors = []
    
    for i in range(n - delta):
        # 计算相对变换
        T1_rel = np.linalg.inv(traj1['poses'][i]) @ traj1['poses'][i + delta]
        T2_rel = np.linalg.inv(traj2['poses'][i]) @ traj2['poses'][i + delta]
        
        # 计算误差变换
        E = np.linalg.inv(T1_rel) @ T2_rel
        
        # 平移误差
        trans_error = np.linalg.norm(E[:3, 3])
        trans_errors.append(trans_error)
        
        # 旋转误差 (角度)
        R_error = E[:3, :3]
        trace = np.trace(R_error)
        trace = np.clip(trace, -1.0, 3.0)  # 防止数值误差
        rot_error = np.arccos((trace - 1.0) / 2.0)
        rot_errors.append(np.degrees(rot_error))
    
    return {
        'rpe_trans_rmse': np.sqrt(np.mean(np.array(trans_errors)**2)),
        'rpe_trans_mean': np.mean(trans_errors),
        'rpe_trans_std': np.std(trans_errors),
        'rpe_rot_rmse': np.sqrt(np.mean(np.array(rot_errors)**2)),
        'rpe_rot_mean': np.mean(rot_errors),
        'rpe_rot_std': np.std(rot_errors),
        'trans_errors': trans_errors,
        'rot_errors': rot_errors
    }


def compute_drift(traj):
    """
    计算轨迹漂移
    """
    positions = traj['positions']
    if len(positions) < 2:
        return {}
    
    # 计算轨迹总长度
    total_length = 0
    for i in range(1, len(positions)):
        total_length += np.linalg.norm(positions[i] - positions[i-1])
    
    # 计算首尾距离（如果是闭环，应该接近0）
    start_end_dist = np.linalg.norm(positions[-1] - positions[0])
    
    # 计算包围盒
    mins = np.min(positions, axis=0)
    maxs = np.max(positions, axis=0)
    bbox_size = maxs - mins
    
    return {
        'total_length': total_length,
        'start_end_distance': start_end_dist,
        'drift_ratio': start_end_dist / total_length if total_length > 0 else 0,
        'bbox_min': mins.tolist(),
        'bbox_max': maxs.tolist(),
        'bbox_size': bbox_size.tolist()
    }


def plot_trajectories_2d(traj1, traj2, labels, output_path):
    """
    绘制2D轨迹对比图
    """
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # XY平面
    ax = axes[0]
    ax.plot(traj1['positions'][:, 0], traj1['positions'][:, 1], 'b-', 
            linewidth=1.5, label=labels[0], alpha=0.8)
    ax.plot(traj2['positions'][:, 0], traj2['positions'][:, 1], 'r--', 
            linewidth=1.5, label=labels[1], alpha=0.8)
    ax.scatter([traj1['positions'][0, 0]], [traj1['positions'][0, 1]], 
               c='g', s=100, marker='o', label='Start', zorder=5)
    ax.scatter([traj1['positions'][-1, 0]], [traj1['positions'][-1, 1]], 
               c='k', s=100, marker='x', label='End', zorder=5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY Plane')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    # XZ平面
    ax = axes[1]
    ax.plot(traj1['positions'][:, 0], traj1['positions'][:, 2], 'b-', 
            linewidth=1.5, label=labels[0], alpha=0.8)
    ax.plot(traj2['positions'][:, 0], traj2['positions'][:, 2], 'r--', 
            linewidth=1.5, label=labels[1], alpha=0.8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('XZ Plane')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    # YZ平面
    ax = axes[2]
    ax.plot(traj1['positions'][:, 1], traj1['positions'][:, 2], 'b-', 
            linewidth=1.5, label=labels[0], alpha=0.8)
    ax.plot(traj2['positions'][:, 1], traj2['positions'][:, 2], 'r--', 
            linewidth=1.5, label=labels[1], alpha=0.8)
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('YZ Plane')
    ax.legend()
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'trajectory_2d.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path / 'trajectory_2d.png'}")


def plot_trajectories_3d(traj1, traj2, labels, output_path):
    """
    绘制3D轨迹对比图
    """
    try:
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.plot(traj1['positions'][:, 0], traj1['positions'][:, 1], traj1['positions'][:, 2],
                'b-', linewidth=1.5, label=labels[0], alpha=0.8)
        ax.plot(traj2['positions'][:, 0], traj2['positions'][:, 1], traj2['positions'][:, 2],
                'r--', linewidth=1.5, label=labels[1], alpha=0.8)
        
        # 标记起点和终点
        ax.scatter([traj1['positions'][0, 0]], [traj1['positions'][0, 1]], 
                   [traj1['positions'][0, 2]], c='g', s=100, marker='o', label='Start')
        ax.scatter([traj1['positions'][-1, 0]], [traj1['positions'][-1, 1]], 
                   [traj1['positions'][-1, 2]], c='k', s=100, marker='x', label='End')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Trajectory Comparison')
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(output_path / 'trajectory_3d.png', dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved: {output_path / 'trajectory_3d.png'}")
    except Exception as e:
        print(f"Warning: 3D plot skipped due to matplotlib compatibility issue: {e}")
        print("  (This is caused by multiple matplotlib versions installed)")
        # 创建替代的2D俯视图
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(traj1['positions'][:, 0], traj1['positions'][:, 1], 'b-', 
                linewidth=1.5, label=labels[0], alpha=0.8)
        ax.plot(traj2['positions'][:, 0], traj2['positions'][:, 1], 'r--', 
                linewidth=1.5, label=labels[1], alpha=0.8)
        ax.scatter([traj1['positions'][0, 0]], [traj1['positions'][0, 1]], 
                   c='g', s=100, marker='o', label='Start', zorder=5)
        ax.scatter([traj1['positions'][-1, 0]], [traj1['positions'][-1, 1]], 
                   c='k', s=100, marker='x', label='End', zorder=5)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Top-Down View (3D plot unavailable)')
        ax.legend()
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(output_path / 'trajectory_3d.png', dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Saved: {output_path / 'trajectory_3d.png'} (2D fallback)")


def plot_error_analysis(ate_result, rpe_result, output_path):
    """
    绘制误差分析图
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # ATE随帧变化
    ax = axes[0, 0]
    ax.plot(ate_result['errors'], 'b-', linewidth=0.5)
    ax.axhline(y=ate_result['mean'], color='r', linestyle='--', 
               label=f'Mean: {ate_result["mean"]:.4f} m')
    ax.axhline(y=ate_result['ate'], color='g', linestyle='--', 
               label=f'RMSE: {ate_result["ate"]:.4f} m')
    ax.set_xlabel('Frame Index')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('Absolute Trajectory Error (ATE)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # ATE分布直方图
    ax = axes[0, 1]
    ax.hist(ate_result['errors'], bins=50, color='blue', alpha=0.7, edgecolor='black')
    ax.axvline(x=ate_result['mean'], color='r', linestyle='--', label=f'Mean: {ate_result["mean"]:.4f}')
    ax.axvline(x=ate_result['median'], color='g', linestyle='--', label=f'Median: {ate_result["median"]:.4f}')
    ax.set_xlabel('Position Error (m)')
    ax.set_ylabel('Count')
    ax.set_title('ATE Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # RPE平移误差
    if rpe_result.get('trans_errors'):
        ax = axes[1, 0]
        ax.plot(rpe_result['trans_errors'], 'b-', linewidth=0.5)
        ax.axhline(y=rpe_result['rpe_trans_mean'], color='r', linestyle='--',
                   label=f'Mean: {rpe_result["rpe_trans_mean"]:.4f} m')
        ax.set_xlabel('Frame Index')
        ax.set_ylabel('Translation Error (m)')
        ax.set_title('Relative Pose Error - Translation')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # RPE旋转误差
    if rpe_result.get('rot_errors'):
        ax = axes[1, 1]
        ax.plot(rpe_result['rot_errors'], 'b-', linewidth=0.5)
        ax.axhline(y=rpe_result['rpe_rot_mean'], color='r', linestyle='--',
                   label=f'Mean: {rpe_result["rpe_rot_mean"]:.4f} deg')
        ax.set_xlabel('Frame Index')
        ax.set_ylabel('Rotation Error (deg)')
        ax.set_title('Relative Pose Error - Rotation')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path / 'error_analysis.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path / 'error_analysis.png'}")


def generate_report(traj1, traj2, labels, ate_result, rpe_result, drift1, drift2, output_path):
    """
    生成详细的差异报告
    """
    report_lines = []
    report_lines.append("=" * 70)
    report_lines.append("FASTLIO2 ROS2 轨迹对比分析报告")
    report_lines.append("=" * 70)
    report_lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append("")
    
    report_lines.append("-" * 70)
    report_lines.append("1. 轨迹基本信息")
    report_lines.append("-" * 70)
    report_lines.append(f"  轨迹1 ({labels[0]}): {traj1['num_poses']} 帧")
    report_lines.append(f"  轨迹2 ({labels[1]}): {traj2['num_poses']} 帧")
    report_lines.append("")
    
    report_lines.append("-" * 70)
    report_lines.append("2. 轨迹范围统计")
    report_lines.append("-" * 70)
    report_lines.append(f"  轨迹1:")
    report_lines.append(f"    - 总长度: {drift1['total_length']:.3f} m")
    report_lines.append(f"    - 首尾距离: {drift1['start_end_distance']:.3f} m")
    report_lines.append(f"    - 漂移比例: {drift1['drift_ratio']*100:.2f}%")
    report_lines.append(f"    - 包围盒: X[{drift1['bbox_min'][0]:.2f}, {drift1['bbox_max'][0]:.2f}] "
                       f"Y[{drift1['bbox_min'][1]:.2f}, {drift1['bbox_max'][1]:.2f}] "
                       f"Z[{drift1['bbox_min'][2]:.2f}, {drift1['bbox_max'][2]:.2f}]")
    report_lines.append(f"  轨迹2:")
    report_lines.append(f"    - 总长度: {drift2['total_length']:.3f} m")
    report_lines.append(f"    - 首尾距离: {drift2['start_end_distance']:.3f} m")
    report_lines.append(f"    - 漂移比例: {drift2['drift_ratio']*100:.2f}%")
    report_lines.append(f"    - 包围盒: X[{drift2['bbox_min'][0]:.2f}, {drift2['bbox_max'][0]:.2f}] "
                       f"Y[{drift2['bbox_min'][1]:.2f}, {drift2['bbox_max'][1]:.2f}] "
                       f"Z[{drift2['bbox_min'][2]:.2f}, {drift2['bbox_max'][2]:.2f}]")
    report_lines.append("")
    
    report_lines.append("-" * 70)
    report_lines.append("3. 绝对轨迹误差 (ATE)")
    report_lines.append("-" * 70)
    report_lines.append(f"  RMSE: {ate_result['ate']:.6f} m")
    report_lines.append(f"  Mean: {ate_result['mean']:.6f} m")
    report_lines.append(f"  Std:  {ate_result['std']:.6f} m")
    report_lines.append(f"  Max:  {ate_result['max']:.6f} m")
    report_lines.append(f"  Min:  {ate_result['min']:.6f} m")
    report_lines.append(f"  Median: {ate_result['median']:.6f} m")
    report_lines.append("")
    
    report_lines.append("-" * 70)
    report_lines.append("4. 相对位姿误差 (RPE)")
    report_lines.append("-" * 70)
    report_lines.append(f"  平移误差:")
    report_lines.append(f"    - RMSE: {rpe_result['rpe_trans_rmse']:.6f} m")
    report_lines.append(f"    - Mean: {rpe_result['rpe_trans_mean']:.6f} m")
    report_lines.append(f"    - Std:  {rpe_result['rpe_trans_std']:.6f} m")
    report_lines.append(f"  旋转误差:")
    report_lines.append(f"    - RMSE: {rpe_result['rpe_rot_rmse']:.6f} deg")
    report_lines.append(f"    - Mean: {rpe_result['rpe_rot_mean']:.6f} deg")
    report_lines.append(f"    - Std:  {rpe_result['rpe_rot_std']:.6f} deg")
    report_lines.append("")
    
    report_lines.append("-" * 70)
    report_lines.append("5. 质量评估")
    report_lines.append("-" * 70)
    
    # 评估回环效果
    if drift1['drift_ratio'] < drift2['drift_ratio']:
        improvement = (drift2['drift_ratio'] - drift1['drift_ratio']) / drift2['drift_ratio'] * 100
        report_lines.append(f"  回环优化效果: 漂移减少 {improvement:.1f}%")
    else:
        degradation = (drift1['drift_ratio'] - drift2['drift_ratio']) / drift2['drift_ratio'] * 100
        report_lines.append(f"  警告: 优化后漂移增加 {degradation:.1f}%，可能存在回环误匹配")
    
    # ATE质量评估
    if ate_result['ate'] < 0.1:
        report_lines.append(f"  轨迹一致性: 优秀 (ATE < 0.1m)")
    elif ate_result['ate'] < 0.5:
        report_lines.append(f"  轨迹一致性: 良好 (ATE < 0.5m)")
    elif ate_result['ate'] < 1.0:
        report_lines.append(f"  轨迹一致性: 一般 (ATE < 1.0m)")
    else:
        report_lines.append(f"  轨迹一致性: 较差 (ATE >= 1.0m)，建议检查回环参数")
    
    report_lines.append("")
    report_lines.append("=" * 70)
    report_lines.append("报告生成完毕")
    report_lines.append("=" * 70)
    
    # 保存报告
    report_text = "\n".join(report_lines)
    report_file = output_path / 'trajectory_comparison_report.txt'
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(report_text)
    print(f"Saved: {report_file}")
    
    # 同时输出到控制台
    print("\n" + report_text)
    
    # 保存JSON格式结果
    results_json = {
        'traj1_info': {
            'label': labels[0],
            'num_poses': traj1['num_poses'],
            'drift': drift1
        },
        'traj2_info': {
            'label': labels[1],
            'num_poses': traj2['num_poses'],
            'drift': drift2
        },
        'ate': {
            'rmse': ate_result['ate'],
            'mean': ate_result['mean'],
            'std': ate_result['std'],
            'max': ate_result['max'],
            'min': ate_result['min'],
            'median': ate_result['median']
        },
        'rpe': {
            'trans_rmse': rpe_result['rpe_trans_rmse'],
            'trans_mean': rpe_result['rpe_trans_mean'],
            'trans_std': rpe_result['rpe_trans_std'],
            'rot_rmse': rpe_result['rpe_rot_rmse'],
            'rot_mean': rpe_result['rpe_rot_mean'],
            'rot_std': rpe_result['rpe_rot_std']
        }
    }
    
    json_file = output_path / 'trajectory_comparison_results.json'
    with open(json_file, 'w') as f:
        json.dump(results_json, f, indent=2)
    print(f"Saved: {json_file}")


def main():
    parser = argparse.ArgumentParser(description='FASTLIO2 ROS2 轨迹对比分析工具')
    parser.add_argument('--traj1', type=str, required=True,
                       help='第一个轨迹文件路径（KITTI格式）')
    parser.add_argument('--traj2', type=str, required=True,
                       help='第二个轨迹文件路径（KITTI格式）')
    parser.add_argument('--label1', type=str, default='Optimized',
                       help='第一个轨迹的标签')
    parser.add_argument('--label2', type=str, default='Original',
                       help='第二个轨迹的标签')
    parser.add_argument('--output', type=str, required=True,
                       help='输出文件夹路径')
    
    args = parser.parse_args()
    
    # 创建输出目录
    output_path = Path(args.output)
    output_path.mkdir(parents=True, exist_ok=True)
    
    print(f"Loading trajectory 1: {args.traj1}")
    traj1 = load_kitti_trajectory(args.traj1)
    print(f"  Loaded {traj1['num_poses']} poses")
    
    print(f"Loading trajectory 2: {args.traj2}")
    traj2 = load_kitti_trajectory(args.traj2)
    print(f"  Loaded {traj2['num_poses']} poses")
    
    labels = [args.label1, args.label2]
    
    # 计算误差指标
    print("\nComputing metrics...")
    ate_result = compute_ate(traj1, traj2)
    rpe_result = compute_rpe(traj1, traj2)
    drift1 = compute_drift(traj1)
    drift2 = compute_drift(traj2)
    
    # 生成可视化
    print("\nGenerating visualizations...")
    plot_trajectories_2d(traj1, traj2, labels, output_path)
    plot_trajectories_3d(traj1, traj2, labels, output_path)
    plot_error_analysis(ate_result, rpe_result, output_path)
    
    # 生成报告
    print("\nGenerating report...")
    generate_report(traj1, traj2, labels, ate_result, rpe_result, drift1, drift2, output_path)
    
    print(f"\nAll results saved to: {output_path}")


if __name__ == '__main__':
    main()
