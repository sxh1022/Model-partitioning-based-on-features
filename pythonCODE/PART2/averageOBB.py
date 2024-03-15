# 根据OBB包围盒进行均匀分块
import bpy
import numpy as np
import random
from mathutils import Matrix
import bmesh
import json

def minimum_bounding_box_pca(points):
    # 计算协方差矩阵
    cov_mat = np.cov(points, rowvar=False, bias=True)
    eig_vals, eig_vecs = np.linalg.eigh(cov_mat)

    # 获取主成分，这些主成分构成了外包矩形的基
    change_of_basis_mat = eig_vecs
    inv_change_of_basis_mat = np.linalg.inv(change_of_basis_mat)

    # 将点云数据转换到主成分坐标系
    aligned = points.dot(inv_change_of_basis_mat.T)

    # 计算外包矩形的最小和最大点
    bb_min = aligned.min(axis=0)
    bb_max = aligned.max(axis=0)

    # 计算外包矩形的中心点并将其转换回世界坐标系
    center = (bb_max + bb_min) / 2
    center_world = center.dot(change_of_basis_mat.T)

    # 计算外包矩形的轴（axes）和尺寸（size）
    axes = eig_vecs  # 主成分构成的轴
    size = bb_max - bb_min  # 外包矩形的尺寸

    return center_world, axes, size


# 获取当前选中的物体
obj = bpy.context.object
mesh = obj.data

# 获取原物体的顶点数据
verts = obj.data.vertices
vert_co_arr = np.empty(len(verts) * 3)
verts.foreach_get("co", vert_co_arr)
vert_co_arr.shape = len(verts), 3

# 获取物体的世界变换矩阵
world_matrix = obj.matrix_world

# 计算包围盒的中心、三个轴和大小
obb_center, obb_axes, obb_size = minimum_bounding_box_pca(vert_co_arr)

print("包围盒中心:", obb_center)
print("包围盒三个轴:", obb_axes)
print("包围盒大小:", obb_size)


# 转换到OBB坐标系下
def world_to_obb(point, obb_center, obb_axes):
    return np.dot(point - obb_center, np.linalg.inv(obb_axes).T)

n = 3

longest_edge_index = np.argmax(obb_size)

# 计算分割点
split_points = np.linspace(obb_center[longest_edge_index] - obb_size[longest_edge_index] / 2, obb_center[longest_edge_index] + obb_size[longest_edge_index] / 2, n + 1)[1:-1]
print("split_points:",split_points)

# 存储每一块中的面片索引
faces_in_partitions = [[] for _ in range(n)]

for face in mesh.polygons:
    # 计算面片的中心点
    vertices = [mesh.vertices[i].co for i in face.vertices]
    centroid = np.mean(vertices, axis=0)
    face_center = world_to_obb(centroid, obb_center, obb_axes)

    # 查找所属的分区
    partition_index = None
    for i, split_point in enumerate(split_points):
        if face_center[longest_edge_index] <= split_point:
            partition_index = i
            break

    # 如果面片在最后一个分区之外，将其分配到最后一个分区
    if partition_index is None:
        partition_index = n - 1
#    print("partition_index:",partition_index)
    faces_in_partitions[partition_index].append(face.index)

for i, partition in enumerate(faces_in_partitions):
    print(f"Partition {i}: Faces {len(partition)}")

def assignMaterials(mesh, k, idx):
    """Assigns a random colored material for each found segment"""
    # k:分割块数   idx:索引列表，其中的每个值表示对应面（Polygon）所属的分割块的索引。列表的长度应与Mesh的面的数量相同。
    print("assignMaterials")

    # clear all existing materials
    mesh.materials.clear()

    for i in range(k):
        material = bpy.data.materials.new(''.join(['mat', mesh.name, str(i)]))
        material.diffuse_color = (random.random(), random.random(), random.random(), 1.0)
        mesh.materials.append(material)

    for i, id in enumerate(idx):
        mesh.polygons[i].material_index = id

# 为每个面片设置对应的分割块索引
face_cluster_indices = [-1] * len(mesh.polygons)
for i, faces_in_partition in enumerate(faces_in_partitions):
    print(len(faces_in_partition))
    for face_index in faces_in_partition:
        face_cluster_indices[face_index] = i

# 调用 assignMaterials 函数
assignMaterials(mesh, len(faces_in_partitions), face_cluster_indices)

# 存储为 JSON 文件的路径
output_file_path = "export/clusters_Avg.json"

# 将 faces_in_partitions 列表导出为 JSON 格式
with open(output_file_path, 'w') as json_file:
    json.dump(faces_in_partitions, json_file)


# 获取模型表面的法向量
normals = [vertex.normal for vertex in mesh.vertices]

# 计算法向量的平均值，作为模型的正面朝向向量
front_vector = np.mean(normals, axis=0)
front_vector /= np.linalg.norm(front_vector)  # 确保向量是单位向量

# 包围盒的轴
obb_axes = np.array(obb_axes)

# 根据包围盒的轴，将正面向量与其匹配
for axis in obb_axes:
    dot_product = np.dot(front_vector, axis)
    if abs(dot_product) > 0.9:  # 阈值可以根据需要调整
        front_vector = axis
        break

# 从左到右的方向向量，与正面向量和包围盒的轴垂直
right_vector = np.cross(front_vector, obb_axes[0])
right_vector /= np.linalg.norm(right_vector)

# 从上到下的方向向量，与正面向量和包围盒的轴垂直
up_vector = np.cross(front_vector, right_vector)
up_vector /= np.linalg.norm(up_vector)

print("模型正面朝向向量:", front_vector)
print("从左到右的方向向量:", right_vector)
print("从上到下的方向向量:", up_vector)
