# 按照曲率分块
import bpy
import bmesh
import random
import math
import mathutils
import itertools
import numpy as np
from mathutils import Vector
from mathutils import Matrix
from sklearn.cluster import KMeans
import json

# KMeans聚类需要提前告知希望按照曲率分块的数量
num_clusters = 3

# 获取活动对象
obj = bpy.context.active_object

# 获取网格数据
mesh = obj.data
bpy.ops.object.mode_set(mode='OBJECT')

bm = bmesh.new()
bm.from_mesh(mesh)
bm.faces.ensure_lookup_table()
bm.edges.ensure_lookup_table()
bm.verts.ensure_lookup_table()


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
    point_np = np.array(point)
    return np.dot(point_np - obb_center, np.linalg.inv(obb_axes).T)


# 找到最长的边的索引
longest_edge_index = np.argmax(obb_size)
longest_edge_direction = obb_axes[longest_edge_index]
print("longest_edge_index:", longest_edge_index)

# 计算曲率
curvatures = []


def calculate_curvature(vertex):
    # 获取顶点周围的法线
    normals = [loop.vert.normal for loop in vertex.link_loops]

    # 计算法线的平均值
    normal_sum = mathutils.Vector((0, 0, 0))
    for normal in normals:
        normal_sum += normal

    normal_avg = normal_sum / len(normals)

    # 计算曲率
    curvature = normal_avg.length
    return curvature


for i, vertex in enumerate(bm.verts):
    if not any(math.isnan(c) for c in vertex.co):
        curvature = calculate_curvature(vertex)
        curvatures.append((i, curvature))

print("curvatures:", len(curvatures))

# 创建一个曲率变化阈值，曲率变化小，基本都在1左右
curvature_threshold = 1

# 找到曲率变化大于阈值的点
split_vertex_indices = [i for i, curvature in curvatures if curvature > curvature_threshold]

print("split_vertex_indices:", len(split_vertex_indices))
# 获取分割点的坐标
split_points = [world_to_obb(bm.verts[i].co, obb_center, obb_axes)[longest_edge_index] for i in split_vertex_indices]


# 合并接近的点
def merge_point(split_points):
    # 定义一个用于合并点的阈值，根据需要更改此值
    merge_threshold = 0.01

    # 初始化一个新的列表来存储合并后的点及其平均曲率
    merged_points = []

    # 遍历split_points列表
    while split_points:
        # 从split_points中取出第一个点作为参考点
        reference_point = split_points.pop(0)

        # 初始化一个列表来存储需要合并的点
        points_to_merge = [reference_point]

        # 初始化一个列表来存储需要合并的点的曲率
        curvatures_to_merge = []

        # 遍历剩余的split_points，查找距离reference_point在最长边方向上小于阈值的点
        i = 0
        while i < len(split_points):
            point = split_points[i]

            # 计算最长边上的距离
            #        vector_between_points = reference_point - point
            #        distance_along_longest_edge = np.dot(vector_between_points, longest_edge_direction)
            distance_along_longest_edge = abs(reference_point - point)

            if distance_along_longest_edge < merge_threshold:
                # 将点添加到合并列表中，并从split_points中移除
                points_to_merge.append(split_points.pop(i))
            else:
                i += 1

        # 计算合并后的点的坐标
        merged_point = sum(point for point in points_to_merge) / len(points_to_merge)

        # 添加合并后的点到merged_points列表
        merged_points.append(merged_point)

    return merged_points


# 合并接近的点
split_points = merge_point(split_points)
n = len(split_points)
print("n:", n)

# 存储每一块中的面片索引
faces_in_partitions = [[] for _ in range(n)]

# 遍历所有面片，将它们分配到不同的块中
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

    faces_in_partitions[partition_index].append(face.index)

# 去掉空的分区
faces_in_partitions = [partition for partition in faces_in_partitions if partition]

for i, partition in enumerate(faces_in_partitions):
    print(f"Partition {i}: Faces {len(partition)}")

# ==========================================================================================
# 找到分块的边界
# 存储每个分区的边缘信息的字典
partition_edges = {}

# 找到分块的边界
for edge in bm.edges:
    adjacent_faces = edge.link_faces
    if len(adjacent_faces) == 2:
        face1_partition = None
        face2_partition = None
        for i, partition in enumerate(faces_in_partitions):
            if adjacent_faces[0].index in partition:
                face1_partition = i
            if adjacent_faces[1].index in partition:
                face2_partition = i
        if face1_partition is not None and face2_partition is not None and face1_partition != face2_partition:
            # 确保 partition1_index < partition2_index
            partition1_index, partition2_index = sorted([face1_partition, face2_partition])

            # 初始化字典中的列表（如果不存在）
            if (partition1_index, partition2_index) not in partition_edges:
                partition_edges[(partition1_index, partition2_index)] = []

            # 将边缘索引添加到相应的分区
            partition_edges[(partition1_index, partition2_index)].append(edge.index)

partition_edges = dict(sorted(partition_edges.items(), key=lambda item: item[0][0]))
# 输出每个分区的边缘信息
for (partition1_index, partition2_index), edges in partition_edges.items():
    print(f"Boundary Partitions {partition1_index} and {partition2_index} Boundary Edges: {len(edges)}")

# ============================================================================================

# 存储每个分区的平均法向量
partition_normals = []

# 遍历所有分区
for faces_in_partition in faces_in_partitions:
    partition_normal_sum = mathutils.Vector((0, 0, 0))

    # 遍历分区中的每个面片
    for face_index in faces_in_partition:
        face = bm.faces[face_index]
        partition_normal_sum += face.normal

    # 计算平均法向量
    partition_normal_avg = partition_normal_sum / len(faces_in_partition)

    # 将平均法向量添加到列表中
    partition_normals.append(partition_normal_avg)

# 输出每个分区的平均法向量
for i, normal in enumerate(partition_normals):
    print(f"Partition {i} Average Normal: {normal}")
# ============================================================================================
# 组合数据，每行包括分区的面片索引和对应的平均法向量
block_data = [(i, face_index, normal) for i, (faces, normal) in enumerate(zip(faces_in_partitions, partition_normals))
              for face_index in faces]

# 提取法向量作为输入特征
input_features = [data[2] for data in block_data]

# 创建K均值聚类模型并拟合数据
kmeans = KMeans(n_clusters=num_clusters)
kmeans.fit(input_features)

# 获取每个块所属的聚类
block_clusters = kmeans.labels_

# 初始化clustered_blocks，将块列表添加到对应的聚类
clustered_blocks = {cluster_index: [] for cluster_index in range(num_clusters)}
for i, (partition_index, face_index, _) in enumerate(block_data):
    cluster_index = block_clusters[i]
    clustered_blocks[cluster_index].append((partition_index, face_index))

# 创建一个新的 faces_in_partitions 格式的列表
clustered_faces_in_partitions = [[] for _ in range(num_clusters)]

# 将 clustered_block_list 中的数据按照 cluster_index 进行分组
for cluster_index, block_list in clustered_blocks.items():
    for partition_index, face_index in block_list:
        clustered_faces_in_partitions[cluster_index].append(face_index)

# 输出每个分区中的面片数量
for i, faces_in_partition in enumerate(clustered_faces_in_partitions):
    print(f"Cluster {i}: {len(faces_in_partition)} faces")

# ================================================================================================

# 输入参数，机械臂长度
arm_length = 1

# 存储分割后的子块
subdivided_blocks = []

# 遍历每一块
for i, faces_in_block in enumerate(clustered_faces_in_partitions):
    if not faces_in_block:
        continue

    # 计算当前块的长度
    min_coord_on_longest_edge = float('inf')
    max_coord_on_longest_edge = float('-inf')

    # 遍历块中的面片，找到块在最长边上的最小和最大坐标,投影
    for face_index in faces_in_block:
        face = bm.faces[face_index]
        for vert in face.verts:
            coord_on_longest_edge = vert.co.dot(longest_edge_direction)
            min_coord_on_longest_edge = min(min_coord_on_longest_edge, coord_on_longest_edge)
            max_coord_on_longest_edge = max(max_coord_on_longest_edge, coord_on_longest_edge)

    block_length_on_longest_edge = max_coord_on_longest_edge - min_coord_on_longest_edge

    print("block_length_on_longest_edge:", block_length_on_longest_edge)

    print("num:", block_length_on_longest_edge / arm_length)
    # 根据 arm_length 分割块
    num_subdivisions = math.ceil(block_length_on_longest_edge / arm_length)

    if num_subdivisions > 1:
        # 创建子块列表
        subblock_length = block_length_on_longest_edge / num_subdivisions
        subblocks = [[] for _ in range(num_subdivisions)]

        # 遍历块中的面片，将它们分配到子块
        for face_index in faces_in_block:
            face = bm.faces[face_index]
            face_center = face.calc_center_median()
            coord_on_longest_edge = face_center.dot(longest_edge_direction)

            # 确定面属于哪个子块
            subblock_index = int((coord_on_longest_edge - min_coord_on_longest_edge) / subblock_length)
            subblocks[subblock_index].append(face_index)

        # 将子块添加到主列表中
        subdivided_blocks.extend(subblocks)
    else:
        # 如果不需要进一步分割，直接将当前块添加到主列表中
        subdivided_blocks.append(faces_in_block)

# 输出每个子块的面片数量
for i, subdivision in enumerate(subdivided_blocks):
    print(f"Subdivision {i}: {len(subdivision)} faces")


# ==================================================================================================

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
for i, faces_in_partition in enumerate(subdivided_blocks):
    for face_index in faces_in_partition:
        face_cluster_indices[face_index] = i

# 调用 assignMaterials 函数
assignMaterials(mesh, len(subdivided_blocks), face_cluster_indices)
bm.free()

# 存储为 JSON 文件的路径
output_file_path = "export/clusters_Avg.json"

# 将 faces_in_partitions 列表导出为 JSON 格式
with open(output_file_path, 'w') as json_file:
    json.dump(subdivided_blocks, json_file)