#!/usr/bin/env python3

''' 
Code to visualize the pointcloud with Open3D

Execute with python3 view_pointcloud.py 

Change your pointcloud path 
'''

import open3d as o3d

pcd = o3d.io.read_point_cloud('/home/hassabo/dev_ws/src/MRAC_SENSING_WORKSHOP/ros_sim/view_pointcloud/cloud.ply') # CHANGE TO YOUR PATH!!
o3d.visualization.draw_geometries([pcd])
