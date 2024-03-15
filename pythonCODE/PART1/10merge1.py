# 深度合并1

import bpy
import math
import random
import json
import bmesh
import numpy as np


# bmesh: 计算平均二面角，只处理每个聚类中边缘属于edge_sections的边
def calculate_average_dihedral_angle(edges, bm):
    total_angle = 0
    total_pairs = 0
    for edge_index in edges:
        edge = bm.edges[edge_index]
        linked_faces = [face for face in edge.link_faces if face]

        if len(linked_faces) == 2:
            # 将弧度转换为角度
            angle = math.degrees(linked_faces[0].normal.angle(linked_faces[1].normal))
            total_angle += angle
            total_pairs += 1

    if total_pairs == 0:
        return 0
    return total_angle / total_pairs


# 计算移除一个特定的边缘的概率
def calculate_edge_section_probability(Length_ESi, Length_ESj, Avg_theta_i, Avg_theta_j, beta):
    if (Length_ESi + Length_ESj) == 0 or (Avg_theta_i + Avg_theta_j) == 0:
        return 0
    Pij = beta * (Length_ESi / (Length_ESi + Length_ESj)) + (1 - beta) * (Avg_theta_i / (Avg_theta_i + Avg_theta_j))
    return Pij


# 聚类合并步骤
def merge_clusters(clusters, edge_section_edges, average_dihedral_angles, beta, edge_section_threshold):
    merged_clusters = []
    for i in range(len(clusters)):

        Length_ESi = len(edge_section_edges[i])
        Avg_theta_i = average_dihedral_angles[i]

        for j in range(i + 1, len(clusters)):
            Length_ESj = len(edge_section_edges[j])
            Avg_theta_j = average_dihedral_angles[j]

            # 检查边缘部分是否有共享边
            shared_edge_section = False
            for edge_section_edge in edge_section_edges[i]:
                if edge_section_edge in edge_section_edges[j]:
                    shared_edge_section = True
                    break

            # 计算移除一个特定的边缘的概率
            Pij = calculate_edge_section_probability(Length_ESi, Length_ESj, Avg_theta_i, Avg_theta_j, beta)

            # 检查是否满足合并条件
            if shared_edge_section and Pij > edge_section_threshold:
                print("cluster ", i, " and cluster ", j, " merge")
                merged_faces = clusters[i][0] | clusters[j][0]
                merged_sharp_edges = clusters[i][1] | clusters[j][1]
                merged_cluster = (merged_faces, merged_sharp_edges)
                merged_clusters.append(merged_cluster)

                # 从 clusters 中删除 i 和 j
                clusters.pop(j)
                clusters.pop(i)
                # 添加合并后的聚类到 clusters 中
                clusters.append(merged_cluster)
                return clusters
    return clusters


# 获取活动对象
obj = bpy.context.active_object

# 获取网格数据
mesh = obj.data

# 创建一个BMesh实例
bm = bmesh.new()
bm.from_mesh(mesh)
bm.edges.ensure_lookup_table()
bm.faces.ensure_lookup_table()
bm.verts.ensure_lookup_table()

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
print("before:", len(clusters))


# 字典 存储每个聚类的信息
def get_clusters_info(clusters):
    edge_sections = set()  # 用于存储EdgeSection的边

    # 遍历每个聚类
    for i in range(len(clusters)):
        for edge in clusters[i][1]:
            is_shared_edge = False
            # 在其他聚类中寻找共享的边
            for j in range(len(clusters)):
                if i != j:  # 排除自己的聚类
                    if edge in clusters[j][1]:
                        is_shared_edge = True
                        break  # 找到共享边，跳出循环
            if is_shared_edge:
                edge_sections.add(edge)

    # 字典 存储每个聚类的信息
    edge_section_edges = {}  # 创建一个空的字典来存储每个聚类的 EdgeSection 的边
    average_dihedral_angles = {}  # 创建一个空的字典来存储每个聚类的 平均二面角，只处理每个聚类中边缘属于edge_sections的边

    # 遍历每个聚类
    for i in range(len(clusters)):
        cluster_edges = clusters[i][1]
        edge_section_edges[i] = [edge for edge in cluster_edges if edge in edge_sections]
        average_dihedral_angles[i] = calculate_average_dihedral_angle(edge_section_edges[i], bm)
    return edge_section_edges, average_dihedral_angles


# 参数设置

# 获取自适应参数
def adapt_parameters(bm, clusters, iterations=10):
    # 初始值
    beta = 0.4
    edge_section_threshold = 0.6

    for _ in range(iterations):
        # 获取当前聚类的边缘数量和平均二面角
        edge_section_edges, average_dihedral_angles = get_clusters_info(clusters)
        current_edge_count = len(edge_section_edges)
        current_dihedral_angle = calculate_average_dihedral_angle(edge_section_edges, bm)

        # 调整beta值和edge_section_threshold
        beta, edge_section_threshold = dynamic_parameters(bm, clusters, edge_section_edges, average_dihedral_angles,
                                                          beta, edge_section_threshold,
                                                          target_edge_count=current_edge_count,
                                                          target_dihedral_angle=current_dihedral_angle)

        # 根据beta值和edge_section_threshold进行合并
        new_clusters = merge_clusters(clusters, edge_section_edges, average_dihedral_angles, beta,
                                      edge_section_threshold)

        # 如果找到更好的参数值，则更新最佳值
        if len(new_clusters) == len(clusters):
            break

        clusters = new_clusters

    return beta, edge_section_threshold


def dynamic_parameters(bm, clusters, edge_section_edges, average_dihedral_angles,
                       current_beta, current_edge_section_threshold,
                       target_edge_count, target_dihedral_angle, threshold=0.1):
    current_edge_count = len(edge_section_edges)
    current_dihedral_angle = calculate_average_dihedral_angle(edge_section_edges, bm)

    # 计算边缘数量和平均二面角的相对差异
    edge_count_difference = (current_edge_count - target_edge_count) / target_edge_count
    dihedral_angle_difference = (current_dihedral_angle - target_dihedral_angle) / target_dihedral_angle

    # 根据相对差异调整beta值和edge_section_threshold
    beta_adjustment = edge_count_difference - dihedral_angle_difference
    edge_section_threshold_adjustment = dihedral_angle_difference - edge_count_difference

    # 根据阈值调整beta值和edge_section_threshold
    if beta_adjustment > threshold:
        beta_adjustment = threshold
    elif beta_adjustment < -threshold:
        beta_adjustment = -threshold

    if edge_section_threshold_adjustment > threshold:
        edge_section_threshold_adjustment = threshold
    elif edge_section_threshold_adjustment < -threshold:
        edge_section_threshold_adjustment = -threshold

    # 计算最终的参数值
    adjusted_beta = current_beta + beta_adjustment
    adjusted_edge_section_threshold = current_edge_section_threshold + edge_section_threshold_adjustment

    # 确保参数在合理范围内
    adjusted_beta = max(0, min(1, adjusted_beta))
    adjusted_edge_section_threshold = max(0, min(1, adjusted_edge_section_threshold))

    return adjusted_beta, adjusted_edge_section_threshold


# 获取自适应参数
beta, edge_section_threshold = adapt_parameters(bm, clusters)
print("Adapted Parameters:")
print("beta:", beta)
print("edge_section_threshold:", edge_section_threshold)


# 当得到的 clusters 长度不发生变化时退出循环
previous_clusters_length = len(clusters)

print("cluster length:", len(clusters))

# 打印的聚类信息
for cluster_index, cluster in enumerate(clusters):
    print(f"Cluster {cluster_index}:", "Faces:", len(cluster[0]), "Sharp Edges:", len(cluster[1]))

while True:
    edge_section_edges, average_dihedral_angles = get_clusters_info(clusters)
    new_clusters = merge_clusters(clusters, edge_section_edges, average_dihedral_angles, beta, edge_section_threshold)

    if len(new_clusters) == previous_clusters_length:
        break

    print("cluster length:", len(clusters))
    clusters = new_clusters
    previous_clusters_length = len(clusters)

    # 打印合并后的聚类信息
    for cluster_index, cluster in enumerate(clusters):
        print(f"Cluster {cluster_index}:", "Faces:", len(cluster[0]), "Sharp Edges:", len(cluster[1]))


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
print(len(face_cluster_indices))
# 调用 assignMaterials 函数
assignMaterials(mesh, len(clusters), face_cluster_indices)

# export json
data = []

for cluster_index, (cluster_faces, cluster_sharp_edges) in enumerate(clusters):
    cluster_data = {
        "faces": list(cluster_faces),
        "edge": list(cluster_sharp_edges)
    }
    data.append(cluster_data)

# Export the data to a JSON file
output_file_path = "export/clusters_data_merge1.json"
with open(output_file_path, "w") as json_file:
    json.dump(data, json_file, indent=4)

print("Data exported to", output_file_path)

print("over")
bm.free()