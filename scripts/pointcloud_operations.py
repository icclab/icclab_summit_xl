# -*- coding: utf-8 -*-
import pcl
import numpy as np
import subprocess
import vtk
import rospy
import datetime
from tools import *

def filtering(raw_cloud):
    start_time = datetime.datetime.now()

    filtered_cloud = pcl.PointCloud()
    planes_cloud = pcl.PointCloud()
    filtered_cloud.from_array(np.asarray(raw_cloud, dtype=np.float32))
    pevent("Loaded PointCloud with: " + str(filtered_cloud.size) + " points.")

    pass_fill = filtered_cloud.make_passthrough_filter()
    pass_fill.set_filter_field_name("z")
    pass_fill.set_filter_limits(0, 1)
    filtered_cloud = pass_fill.filter()
    pinfo("PointCloud after max range filtering has: " + str(filtered_cloud.size) + " points.")

    #sor = filtered_cloud.make_voxel_grid_filter()
    #sor.set_leaf_size(0.005, 0.005, 0.005)
    #filtered_cloud = sor.filter()
    #pinfo("Downsampled PointCloud has: " + str(filtered_cloud.size) + " points.")

    stat_fill = filtered_cloud.make_statistical_outlier_filter()
    stat_fill.set_mean_k(50)
    stat_fill.set_std_dev_mul_thresh(1.0)
    filtered_cloud = stat_fill.filter()
    pinfo("PointCloud after outliers filtering has: " + str(filtered_cloud.size) + " points.")

    seg = filtered_cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(1000)
    seg.set_distance_threshold(0.01)
    indices, model = seg.segment()
    planes_cloud = filtered_cloud.extract(indices, negative=False)
    filtered_cloud = filtered_cloud.extract(indices, negative=True)
    pinfo("PointCloud after plane filtering has: " + str(filtered_cloud.size) + " points.")

    # extract objects from filtered cloud which now should contain graspable objects
    tree = filtered_cloud.make_kdtree()
    ec = filtered_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(200000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Filtering returned empty point cloud, return and try again
    if len(cluster_indices) == 0:
        return pcl.PointCloud()

    # probably dont need it, looks like cluster_indices is sorted, so first element is the biggest
    # max_size = 0
    # biggest_element_idx = 0
    # for i, indices in enumerate(cluster_indices):
    #     if max_size < len(indices):
    #         max_size = len(indices)
    #         biggest_element_idx = i
    #
    # print "Biggest el idx: " + str(biggest_element_idx)

    points = np.zeros((len(cluster_indices[0]), 3), dtype=np.float32)
    for j, indice in enumerate(cluster_indices[0]):
        points[j][0] = filtered_cloud[indice][0]
        points[j][1] = filtered_cloud[indice][1]
        points[j][2] = filtered_cloud[indice][2]

    extracted_object_cloud = pcl.PointCloud()
    extracted_object_cloud.from_array(points)
    pinfo("Extracted object PointCloud has: " + str(extracted_object_cloud.size) + " points.")

    # For now I operate on 3 pointclouds:
    #     filtered_cloud - contains filtered raw cloud without planes
    #     planes_cloud - contains planes extracted from filtered raw cloud
    #     extracted_object_cloud - biggest object extracted from filtered_cloud
    #
    # I need to make new one which should contain [planes_cloud + (filtered_cloud - extracted_object_cloud)]
    # and treat it as an obstacles pointcloud

    obstacles_points = [[0., 0., 0.]]

    obstacles_points = np.vstack((obstacles_points, planes_cloud.to_array()))

    # [cloud - extracted_object_cloud] points can be get from cluster_indices
    # but without points listed under cluster_indices[0]
    for indice in range(1, len(cluster_indices)):       # TODO: maybe can be done with extract? https://strawlab.github.io/python-pcl/
        for _, x in enumerate(cluster_indices[indice]):
            obstacles_points = np.vstack((obstacles_points, filtered_cloud[x]))

    obstacles_cloud = pcl.PointCloud()
    obstacles_cloud.from_array(obstacles_points.astype(dtype=np.float32))

    pcl.save(extracted_object_cloud, "objects.pcd")
    pcl.save(obstacles_cloud, "obstacles.pcd")

    # Make a mesh from object pointcloud and save it to stl file to load if from moveit side
    create_mesh_and_save(extracted_object_cloud)

    # Publish cloud with extracted obstacles to create octomap
    pcd = subprocess.Popen(['rosrun', 'pcl_ros', 'pcd_to_pointcloud', 'obstacles.pcd', '_frame_id:=arm_camera_color_optical_frame'])

    #pcd_2 = subprocess.Popen(
    #    ['rosrun', 'pcl_ros', 'pcd_to_pointcloud', 'objects.pcd', 'cloud_pcd2', '_frame_id:=xtion_rgb_optical_frame'])

    pinfo("Pointcloud filtering + obj extraction stage took: " + str(datetime.datetime.now() - start_time))

    # wait for pointcloud to be poublished and then stop
    rospy.sleep(3)
    pcd.terminate()
    #pcd_2.terminate()

    return extracted_object_cloud



def create_mesh_and_save(cloud):
    filename = "object.stl"

    vtk_points = vtk.vtkPoints()
    np_cloud = np.asarray(cloud)

    for i in range(0, np_cloud.shape[0]):
        vtk_points.InsertPoint(i, np_cloud[i][0], np_cloud[i][1], np_cloud[i][2])

    profile = vtk.vtkPolyData()
    profile.SetPoints(vtk_points)

    aCellArray = vtk.vtkCellArray()

    boundary = vtk.vtkPolyData()
    boundary.SetPoints(profile.GetPoints())
    boundary.SetPolys(aCellArray)

    delny = vtk.vtkDelaunay2D()
    delny.SetInputData(profile)
    delny.SetSourceData(boundary)
    delny.SetTolerance(0.0001)
    delny.SetAlpha(4.0)
    delny.SetOffset(1.25)
    delny.BoundingTriangulationOff()
    delny.Update()

    stl_writer = vtk.vtkSTLWriter()
    stl_writer.SetFileName(filename)
    stl_writer.SetInputConnection(delny.GetOutputPort())
    stl_writer.Write()
