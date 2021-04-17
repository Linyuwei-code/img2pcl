import os
import math
import h5py
import json
import argparse
import numpy as np
from tqdm import tqdm

import pymesh
import pyntcloud
import open3d as o3d


def pymesh_shapenet_core_55(input_dir):
    if not os.path.exists(input_dir):
        raise AssertionError('No ShapeNetCorev2 dataset found in the following directory: ' + input_dir)

    def find_class_name(taxonomy_local, class_dir_local):
        for i in range(len(taxonomy_local)):
            if class_dir_local in taxonomy_local[i]['synsetId']:
                return i, taxonomy_local[i]['name']
        return -1, 'NULL'

    with open(os.path.join(input_dir, 'taxonomy.json')) as data_file:
        print(type(data_file))
        taxonomy = json.load(data_file)

    ptclds = {}
    clsnms = []

    classes_dir = [os.path.join(input_dir, el) for el in os.listdir(input_dir) if
                   os.path.isdir(os.path.join(input_dir, el))]

    for idx, class_dir in enumerate(classes_dir):

        class_name = find_class_name(taxonomy, os.path.split(class_dir)[1])
        print('Converting class: ' + class_name[1])
        clsnms.append(class_name[1])
        ptclds[idx] = []

        objcts_dir = [os.path.join(class_dir, el) for el in os.listdir(class_dir) if
                      os.path.isdir(os.path.join(class_dir, el))]

        for objct_dir in tqdm(objcts_dir):

            object_path = os.path.join(os.path.join(objct_dir, 'models'), 'model_normalized.obj')
            pymesh_path = os.path.join('./ShapeNetPymesh', os.path.split(class_dir)[1])
            print(pymesh_path)

            if not os.path.exists(object_path):
                print('No OBJ path at: ' + object_path)
                continue

            if not os.path.exists(pymesh_path):
                os.mkdir(pymesh_path)

            try:
                mesh = pymesh.load_mesh(object_path)
                pymesh.save_mesh(os.path.join(pymesh_path, os.path.split(objct_dir)[-1] + '.ply'), mesh)
            except ValueError:
                print("An exception occurred: " + object_path)
                exit()


def sample_shapenet_core_55(input_dir, out_dir, point_cloud_size):
    if not os.path.exists(input_dir):
        raise AssertionError('No ShapeNetPymesh dataset found in the following directory: ' + input_dir)

    classes_dir = [os.path.join(input_dir, el) for el in os.listdir(input_dir) if
                   os.path.isdir(os.path.join(input_dir, el))]

    for idx, class_dir in enumerate(classes_dir):
        objcts_dir = [os.path.join(class_dir, el) for el in os.listdir(class_dir)]

        for objct_dir in tqdm(objcts_dir):
            object_path = objct_dir
            pclpcd_path = os.path.join(os.path.join('./ShapeNetSample', str(point_cloud_size)),
                                       os.path.split(class_dir)[-1])

            if not os.path.exists(object_path):
                print('No OBJ path at: ' + object_path)
                continue

            if not os.path.exists(pclpcd_path):
                os.mkdir(pclpcd_path)

            try:
                pynt = pyntcloud.PyntCloud.from_file(object_path)
                cloud = pynt.get_sample('mesh_random', n=point_cloud_size)
                cloud = cloud.values
                #print(cloud)
                """
                for dim in [0, 1, 2]:
                    dim_mean = np.mean(cloud[: dim])
                    cloud[:, dim] -= dim_mean

                distances = [np.linalg.norm(point) for point in cloud]
                scale = 1.0 / np.max(distances)
                cloud *= scale
                """

                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(cloud)
                o3d.io.write_point_cloud(
                    os.path.join(pclpcd_path, os.path.split(objct_dir)[-1].rstrip('.ply') + '.pcd'), point_cloud)

            except ValueError:
                print("An exception occurred: " + object_path)
                exit()


if __name__ == '__main__':
    input_dir = 'ShapeNetPymesh'
    output_dir = None
    point_cloud_size = 16384
    # pymesh_shapenet_core_55(input_dir)
    sample_shapenet_core_55(input_dir, output_dir, point_cloud_size)
