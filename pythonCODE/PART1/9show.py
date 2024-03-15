# 和7一样，为不同聚类显示不同的颜色

import bpy
import math
import random
import json

# 获取活动对象
obj = bpy.context.active_object

# 获取网格数据
mesh = obj.data

# 文件路径
input_file_path = "export/clusters_data1.json"

# 读取JSON文件
with open(input_file_path, "r") as json_file:
    json_data = json.load(json_file)

# 重构clusters
clusters = []
for cluster_data in json_data:
    cluster_faces = set(cluster_data["faces"])
    #    cluster_sharp_edges = set(tuple(edge) for edge in cluster_data["edge"])
    cluster_sharp_edges = set(cluster_data["edge"])
    clusters.append((cluster_faces, cluster_sharp_edges))

print("read file")
print("cluster num:", len(clusters))


def assignMaterials(mesh, k, idx):
    """Assigns a random colored material for each found segment"""
    # k:分割块数   idx:索引列表，其中的每个值表示对应面（Polygon）所属的分割块的索引。列表的长度应与Mesh的面的数量相同。
    print("assignMaterials")

    # clear all existing materials
    mesh.materials.clear()

    for i in range(k):
        material = bpy.data.materials.new(''.join(['mat', mesh.name, str(i)]))
        material.diffuse_color = (random.random(), random.random(),
                                  random.random(), 1.0)
        mesh.materials.append(material)

    for i, id in enumerate(idx):
        mesh.polygons[i].material_index = id

    # 为每个面片设置对应的分割块索引


face_cluster_indices = [-1] * len(mesh.polygons)
for cluster_index, (cluster_faces, cluster_sharp_edges) in enumerate(clusters):
    for face_index in cluster_faces:
        face_cluster_indices[face_index] = cluster_index

# 调用 assignMaterials 函数
assignMaterials(mesh, len(clusters), face_cluster_indices)
print("cluster num:", len(clusters))
print("over")