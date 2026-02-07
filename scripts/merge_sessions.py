#!/usr/bin/env python3
"""
Session Merge Tool for FASTLIO2
Manually merge multiple mapping sessions into a unified map.

Usage:
    python3 merge_sessions.py --sessions /path/to/session1 /path/to/session2 --output /path/to/merged
    python3 merge_sessions.py --sessions session1 session2 --output merged --merge-pcd

Features:
    - Merge pose graphs (.g2o files) with ID offset
    - Merge ScanContext descriptors (.scd files)
    - Optionally merge point cloud maps (.pcd files)
    - Generate unified trajectory files
"""

import os
import sys
import argparse
import shutil
import numpy as np
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import re

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Warning: open3d not installed. PCD merging will use PCL fallback.")


class SessionData:
    """Data structure for a single mapping session"""
    def __init__(self, path: str):
        self.path = Path(path)
        self.name = self.path.name
        self.vertices: List[Dict] = []  # List of {id, x, y, z, qx, qy, qz, qw}
        self.edges: List[Dict] = []     # List of edge data
        self.scd_files: List[Path] = []
        self.poses: List[np.ndarray] = []  # 4x4 transformation matrices
        self.num_keyframes = 0
        
    def load(self) -> bool:
        """Load session data from directory"""
        if not self.path.exists():
            print(f"Error: Session path does not exist: {self.path}")
            return False
            
        # Load pose graph
        g2o_file = self.path / "pose_graph.g2o"
        if g2o_file.exists():
            self._load_g2o(g2o_file)
        else:
            print(f"Warning: No pose_graph.g2o found in {self.path}")
            
        # Load poses from trajectory file
        pose_file = self.path / "optimized_pose.txt"
        if pose_file.exists():
            self._load_poses(pose_file)
            
        # Find SCD files
        scd_dir = self.path / "scd"
        if scd_dir.exists():
            self.scd_files = sorted(scd_dir.glob("*.scd"))
            
        self.num_keyframes = max(len(self.vertices), len(self.poses), len(self.scd_files))
        print(f"Loaded session '{self.name}': {self.num_keyframes} keyframes, "
              f"{len(self.edges)} edges, {len(self.scd_files)} SCD files")
        return True
        
    def _load_g2o(self, filepath: Path):
        """Load g2o format pose graph"""
        is_loop = False
        loop_fitness = 0.0
        
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    # Check for loop marker
                    if '# LOOP' in line:
                        is_loop = True
                        match = re.search(r'fitness=([\d.]+)', line)
                        if match:
                            loop_fitness = float(match.group(1))
                    continue
                    
                parts = line.split()
                if parts[0] == 'VERTEX_SE3:QUAT':
                    vertex = {
                        'id': int(parts[1]),
                        'x': float(parts[2]),
                        'y': float(parts[3]),
                        'z': float(parts[4]),
                        'qx': float(parts[5]),
                        'qy': float(parts[6]),
                        'qz': float(parts[7]),
                        'qw': float(parts[8])
                    }
                    self.vertices.append(vertex)
                    
                elif parts[0] == 'EDGE_SE3:QUAT':
                    edge = {
                        'from_id': int(parts[1]),
                        'to_id': int(parts[2]),
                        'x': float(parts[3]),
                        'y': float(parts[4]),
                        'z': float(parts[5]),
                        'qx': float(parts[6]),
                        'qy': float(parts[7]),
                        'qz': float(parts[8]),
                        'qw': float(parts[9]),
                        'info': [float(x) for x in parts[10:31]],
                        'is_loop': is_loop,
                        'fitness': loop_fitness
                    }
                    self.edges.append(edge)
                    is_loop = False
                    loop_fitness = 0.0
                    
    def _load_poses(self, filepath: Path):
        """Load KITTI format poses"""
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                values = [float(x) for x in line.split()]
                if len(values) == 12:
                    # KITTI format: R(3x3) flattened row-major + t(3)
                    pose = np.eye(4)
                    pose[0, :3] = values[0:3]
                    pose[0, 3] = values[3]
                    pose[1, :3] = values[4:7]
                    pose[1, 3] = values[7]
                    pose[2, :3] = values[8:11]
                    pose[2, 3] = values[11]
                    self.poses.append(pose)


class SessionMerger:
    """Merge multiple mapping sessions"""
    
    def __init__(self, output_path: str):
        self.output_path = Path(output_path)
        self.sessions: List[SessionData] = []
        self.id_offsets: List[int] = []  # ID offset for each session
        
    def add_session(self, session_path: str) -> bool:
        """Add a session to merge"""
        session = SessionData(session_path)
        if session.load():
            self.sessions.append(session)
            return True
        return False
        
    def merge(self, merge_pcd: bool = False, voxel_size: float = 0.1) -> bool:
        """Perform the merge operation"""
        if len(self.sessions) < 1:
            print("Error: Need at least 1 session to merge")
            return False
            
        # Create output directory
        self.output_path.mkdir(parents=True, exist_ok=True)
        (self.output_path / "scd").mkdir(exist_ok=True)
        (self.output_path / "pcd").mkdir(exist_ok=True)
        
        # Calculate ID offsets
        self._calculate_offsets()
        
        # Merge pose graph
        self._merge_pose_graph()
        
        # Merge SCD files
        self._merge_scd_files()
        
        # Merge poses
        self._merge_poses()
        
        # Optionally merge PCD files
        if merge_pcd:
            self._merge_pcd_files(voxel_size)
            
        # Generate metadata
        self._generate_metadata()
        
        print(f"\nMerge completed! Output saved to: {self.output_path}")
        return True
        
    def _calculate_offsets(self):
        """Calculate ID offsets for each session"""
        self.id_offsets = [0]
        cumulative = 0
        for i, session in enumerate(self.sessions[:-1]):
            cumulative += session.num_keyframes
            self.id_offsets.append(cumulative)
            
        print(f"ID offsets: {self.id_offsets}")
        
    def _merge_pose_graph(self):
        """Merge all pose graphs into one g2o file"""
        output_file = self.output_path / "pose_graph.g2o"
        
        total_vertices = sum(len(s.vertices) for s in self.sessions)
        total_edges = sum(len(s.edges) for s in self.sessions)
        
        with open(output_file, 'w') as f:
            f.write("# FASTLIO2 Merged Pose Graph\n")
            f.write(f"# Sessions: {len(self.sessions)}\n")
            f.write(f"# Total Vertices: {total_vertices}\n")
            f.write(f"# Total Edges: {total_edges}\n\n")
            
            # Write vertices with offset IDs
            for session_idx, session in enumerate(self.sessions):
                offset = self.id_offsets[session_idx]
                f.write(f"# Session {session_idx}: {session.name} (offset={offset})\n")
                
                for v in session.vertices:
                    new_id = v['id'] + offset
                    f.write(f"VERTEX_SE3:QUAT {new_id} "
                           f"{v['x']:.9f} {v['y']:.9f} {v['z']:.9f} "
                           f"{v['qx']:.9f} {v['qy']:.9f} {v['qz']:.9f} {v['qw']:.9f}\n")
                           
            f.write("\n")
            
            # Write edges with offset IDs
            for session_idx, session in enumerate(self.sessions):
                offset = self.id_offsets[session_idx]
                
                for e in session.edges:
                    new_from = e['from_id'] + offset
                    new_to = e['to_id'] + offset
                    
                    if e['is_loop']:
                        f.write(f"# LOOP fitness={e['fitness']:.4f}\n")
                        
                    info_str = ' '.join(f"{x:.6f}" for x in e['info'])
                    f.write(f"EDGE_SE3:QUAT {new_from} {new_to} "
                           f"{e['x']:.9f} {e['y']:.9f} {e['z']:.9f} "
                           f"{e['qx']:.9f} {e['qy']:.9f} {e['qz']:.9f} {e['qw']:.9f} "
                           f"{info_str}\n")
                           
        print(f"Merged pose graph: {total_vertices} vertices, {total_edges} edges")
        
    def _merge_scd_files(self):
        """Copy and rename SCD files with new IDs"""
        output_dir = self.output_path / "scd"
        total_copied = 0
        
        for session_idx, session in enumerate(self.sessions):
            offset = self.id_offsets[session_idx]
            
            for scd_file in session.scd_files:
                # Extract original ID from filename
                try:
                    orig_id = int(scd_file.stem)
                except ValueError:
                    continue
                    
                new_id = orig_id + offset
                new_filename = f"{new_id:06d}.scd"
                shutil.copy2(scd_file, output_dir / new_filename)
                total_copied += 1
                
        print(f"Merged SCD files: {total_copied} descriptors")
        
    def _merge_poses(self):
        """Merge and save trajectory files"""
        output_file = self.output_path / "optimized_pose.txt"
        
        with open(output_file, 'w') as f:
            for session in self.sessions:
                for pose in session.poses:
                    # Write KITTI format
                    f.write(f"{pose[0,0]:.9f} {pose[0,1]:.9f} {pose[0,2]:.9f} {pose[0,3]:.9f} "
                           f"{pose[1,0]:.9f} {pose[1,1]:.9f} {pose[1,2]:.9f} {pose[1,3]:.9f} "
                           f"{pose[2,0]:.9f} {pose[2,1]:.9f} {pose[2,2]:.9f} {pose[2,3]:.9f}\n")
                           
        total_poses = sum(len(s.poses) for s in self.sessions)
        print(f"Merged trajectory: {total_poses} poses")
        
    def _merge_pcd_files(self, voxel_size: float):
        """Merge point cloud maps"""
        if HAS_OPEN3D:
            self._merge_pcd_open3d(voxel_size)
        else:
            self._merge_pcd_copy()
            
    def _merge_pcd_open3d(self, voxel_size: float):
        """Merge PCD files using Open3D"""
        merged_cloud = o3d.geometry.PointCloud()
        
        for session in self.sessions:
            # Try to load GlobalMap.pcd
            global_map = session.path / "GlobalMap.pcd"
            if global_map.exists():
                print(f"Loading {global_map}...")
                cloud = o3d.io.read_point_cloud(str(global_map))
                merged_cloud += cloud
            else:
                # Try filterGlobalMap.pcd
                filter_map = session.path / "filterGlobalMap.pcd"
                if filter_map.exists():
                    print(f"Loading {filter_map}...")
                    cloud = o3d.io.read_point_cloud(str(filter_map))
                    merged_cloud += cloud
                    
        if len(merged_cloud.points) > 0:
            # Downsample
            print(f"Downsampling with voxel size {voxel_size}...")
            merged_cloud = merged_cloud.voxel_down_sample(voxel_size)
            
            # Save
            output_file = self.output_path / "GlobalMap.pcd"
            o3d.io.write_point_cloud(str(output_file), merged_cloud)
            
            # Also save filtered version
            filter_file = self.output_path / "filterGlobalMap.pcd"
            o3d.io.write_point_cloud(str(filter_file), merged_cloud)
            
            print(f"Merged point cloud: {len(merged_cloud.points)} points")
        else:
            print("Warning: No point cloud data to merge")
            
    def _merge_pcd_copy(self):
        """Fallback: just copy individual PCD files"""
        output_dir = self.output_path / "pcd"
        total_copied = 0
        
        for session_idx, session in enumerate(self.sessions):
            offset = self.id_offsets[session_idx]
            pcd_dir = session.path / "pcd"
            
            if pcd_dir.exists():
                for pcd_file in sorted(pcd_dir.glob("*.pcd")):
                    try:
                        orig_id = int(pcd_file.stem)
                    except ValueError:
                        continue
                        
                    new_id = orig_id + offset
                    new_filename = f"{new_id:06d}.pcd"
                    shutil.copy2(pcd_file, output_dir / new_filename)
                    total_copied += 1
                    
        print(f"Copied PCD files: {total_copied} clouds")
        
    def _generate_metadata(self):
        """Generate session metadata file"""
        metadata_file = self.output_path / "session_metadata.yaml"
        
        total_keyframes = sum(s.num_keyframes for s in self.sessions)
        total_edges = sum(len(s.edges) for s in self.sessions)
        loop_count = sum(1 for s in self.sessions for e in s.edges if e.get('is_loop', False))
        
        with open(metadata_file, 'w') as f:
            f.write("# FASTLIO2 Merged Session Metadata\n")
            f.write(f"merged_sessions: {len(self.sessions)}\n")
            f.write(f"total_keyframes: {total_keyframes}\n")
            f.write(f"total_edges: {total_edges}\n")
            f.write(f"loop_closures: {loop_count}\n")
            f.write("sessions:\n")
            
            for i, session in enumerate(self.sessions):
                f.write(f"  - name: {session.name}\n")
                f.write(f"    path: {session.path}\n")
                f.write(f"    keyframes: {session.num_keyframes}\n")
                f.write(f"    id_offset: {self.id_offsets[i]}\n")
                
        print(f"Generated metadata: {metadata_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Merge multiple FASTLIO2 mapping sessions",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Merge two sessions
  python3 merge_sessions.py --sessions /path/to/session1 /path/to/session2 --output /path/to/merged
  
  # Merge with point cloud merging
  python3 merge_sessions.py --sessions session1 session2 --output merged --merge-pcd
  
  # Merge with custom voxel size
  python3 merge_sessions.py --sessions s1 s2 s3 --output merged --merge-pcd --voxel-size 0.2
        """
    )
    
    parser.add_argument('--sessions', '-s', nargs='+', required=True,
                       help='Paths to session directories to merge')
    parser.add_argument('--output', '-o', required=True,
                       help='Output directory for merged session')
    parser.add_argument('--merge-pcd', action='store_true',
                       help='Also merge point cloud maps')
    parser.add_argument('--voxel-size', type=float, default=0.1,
                       help='Voxel size for point cloud downsampling (default: 0.1)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("FASTLIO2 Session Merge Tool")
    print("=" * 60)
    
    # Create merger
    merger = SessionMerger(args.output)
    
    # Add sessions
    for session_path in args.sessions:
        if not merger.add_session(session_path):
            print(f"Failed to load session: {session_path}")
            sys.exit(1)
            
    # Perform merge
    if not merger.merge(merge_pcd=args.merge_pcd, voxel_size=args.voxel_size):
        print("Merge failed!")
        sys.exit(1)
        
    print("\nMerge completed successfully!")
    

if __name__ == "__main__":
    main()
