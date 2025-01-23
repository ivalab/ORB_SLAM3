#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file vis_trajs.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 01-22-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""


### Full Example in Open3D

import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation


def create_camera_frame(pose, scale=0.1):
    """
    Create a 3D camera frame using Open3D.
    :param pose: 4x4 numpy array representing the camera pose.
    :param scale: Scale of the camera frame.
    :return: Open3D geometry for the camera frame.
    """
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)
    frame.transform(pose)
    return frame


def plot_cameras_and_trajectory(camera_poses, scale=0.1):
    """
    Plot cameras and their trajectory in Open3D.
    :param camera_poses: List of 4x4 numpy arrays representing camera poses.
    :param scale: Scale of the camera frames.
    """
    geometries = []
    trajectory_points = []

    # Loop through each camera pose
    for pose in camera_poses:
        # Add the camera frame to geometries
        camera_frame = create_camera_frame(pose, scale)
        geometries.append(camera_frame)

        # Extract the camera's position (translation part)
        position = pose[:3, 3]
        trajectory_points.append(position)

    # Create a trajectory line set
    trajectory_line_set = create_trajectory_line_set(trajectory_points)
    geometries.append(trajectory_line_set)

    # Visualize
    o3d.visualization.draw_geometries(geometries)


def create_trajectory_line_set(points):
    """
    Create a line set to visualize the trajectory connecting the points.
    :param points: List of 3D points (Nx3 array or list).
    :return: Open3D LineSet object.
    """
    points = np.array(points)
    lines = [[i, i + 1] for i in range(len(points) - 1)]  # Connect consecutive points
    colors = [[1, 0, 0] for _ in lines]  # Red color for trajectory lines
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def convert_pose_array_2_mat(stamped_poses: np.ndarray):
    poses = []
    for p in stamped_poses:
        mat = np.eye(4)
        mat[:3, :3] = Rotation.from_quat(p[4:]).as_matrix()
        mat[:3, 3] = p[1:4]
        # print(mat)
        poses.append(mat)
    return poses


# Example usage
if __name__ == "__main__":
    # Define some example camera poses
    stamped_poses = np.loadtxt(
        "/mnt/DATA/experiments/semantic/cid/tmp/medium/orb3_rgbd_wo/_Speedx1.0/ObsNumber_1000_Round1/floor13_1_KeyFrameTrajectory.txt",
        ndmin=2,
    )

    mats = convert_pose_array_2_mat(stamped_poses)

    # mats = [
    #     np.array([
    #         [1, 0, 0, i],  # Translation along X
    #         [0, 1, 0, 0],
    #         [0, 0, 1, 0],
    #         [0, 0, 0, 1]
    #     ]) for i in range(5)
    # ]

    plot_cameras_and_trajectory(mats)
