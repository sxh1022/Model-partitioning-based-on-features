#深度合并2

import bpy
import math
import random
import json
import bmesh
import numpy as np


# 计算聚类的周长和面积
def calculate_cluster_perimeter_and_area(cluster_face_indices, cluster_edge_indices, bm):
    perimeter = 0
    area = 0

    if not cluster_edge_indices:
        return perimeter, area

    # 计算周长
    for edge_index in cluster_edge_indices:
        edge = bm.edges[edge_index]
        perimeter += edge.calc_length()

    if not cluster_face_indices:
        return perimeter, area

    # 计算面积
    face_area_sum = sum(bm.faces[face_index].calc_area() for face_index in cluster_face_indices)
    area += face_area_sum

    return perimeter, area


# 计算聚类之间的合并概率
def calculate_merge_probability(rho_i, rho_j, Area_Ci, Area_Cj, gamma):
    # 数值溢出。当 rho_j 的值非常大的时候，math.exp(-rho_j) 可能会变得非常接近零，甚至在计算机内部表示中被截断为零。np.exp(-rho_j)
    Pj_Ci = 1 - gamma * (np.exp(-rho_i) / (np.exp(-rho_i) + np.exp(-rho_j))) - (1 - gamma) * (
                Area_Ci / (Area_Ci + Area_Cj))
    return Pj_Ci


# 聚类合并步骤
def merge_clusters(clusters, edge_section_edges, perimeter_dict, area_dict, rho_dict, gamma, merge_threshold):
    merged_clusters = []
    for i in range(len(clusters)):

        perimeter_i = perimeter_dict[i]
        area_i = area_dict[i]
        rho_i = rho_dict[i]

        for j in range(i + 1, len(clusters)):

            perimeter_j = perimeter_dict[j]
            area_j = area_dict[j]
            rho_j = rho_dict[j]
            Pj_Ci = calculate_merge_probability(rho_i, rho_j, area_i, area_j, gamma)

            # 检查是否存在共享的边缘部分
            shared_edge_section = False
            for edge_section_edge in edge_section_edges[i]:
                if edge_section_edge in edge_section_edges[j]:
                    shared_edge_section = True
                    break

            # 检查是否满足合并条件（共享边和概率）
            if shared_edge_section and Pj_Ci > merge_threshold:
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
input_file_path = "export/clusters_data_merge1.json"

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
    perimeter_dict = {}
    area_dict = {}
    rho_dict = {}  # 聚类的平面性

    # 遍历每个聚类
    for i in range(len(clusters)):
        cluster_edges = clusters[i][1]
        edge_section_edges[i] = [edge for edge in cluster_edges if edge in edge_sections]
        cluster_faces = clusters[i][0]
        cluster_perimeter, cluster_area = calculate_cluster_perimeter_and_area(cluster_faces, cluster_edges, bm)

        # 存储聚类的周长和面积,平面性
        perimeter_dict[i] = cluster_perimeter
        area_dict[i] = cluster_area
        rho_dict[i] = cluster_perimeter ** 2 / (4 * math.pi * cluster_area)  # 计算平面性
    return edge_section_edges, perimeter_dict, area_dict, rho_dict


# 参数设置

# 自适应参数选择
def adapt_parameters(bm, clusters, iterations=10):
    # 初始值
    gamma = 0.2
    merge_threshold = 0.5

    for _ in range(iterations):
        # 获取当前聚类的平面性和面积信息
        edge_section_edges, perimeter_dict, area_dict, rho_dict = get_clusters_info(clusters)

        # 计算当前平面性和面积的中位数
        median_rho = np.median(list(rho_dict.values()))
        median_area = np.median(list(area_dict.values()))

        # 调整 gamma 和 merge_threshold
        gamma, merge_threshold = dynamic_parameters(clusters, gamma, merge_threshold, median_rho, median_area)

        # 根据新的参数进行合并
        new_clusters = merge_clusters(clusters, edge_section_edges, perimeter_dict, area_dict, rho_dict, gamma,
                                      merge_threshold)

        # 如果找到更好的参数值，则更新最佳值
        if len(new_clusters) == len(clusters):
            break

        clusters = new_clusters

    return gamma, merge_threshold


def dynamic_parameters(clusters, current_gamma, current_merge_threshold,
                       target_rho, target_area, threshold=0.1):
    # 计算当前平面性和面积的相对差异
    rho_difference = (target_rho - current_gamma)
    area_difference = (target_area - current_merge_threshold)

    # 根据相对差异调整 gamma 和 merge_threshold
    gamma_adjustment = rho_difference
    merge_threshold_adjustment = area_difference

    # 根据阈值调整 gamma 和 merge_threshold
    if gamma_adjustment > threshold:
        gamma_adjustment = threshold
    elif gamma_adjustment < -threshold:
        gamma_adjustment = -threshold

    if merge_threshold_adjustment > threshold:
        merge_threshold_adjustment = threshold
    elif merge_threshold_adjustment < -threshold:
        merge_threshold_adjustment = -threshold

    # 计算最终的参数值
    adjusted_gamma = current_gamma + gamma_adjustment
    adjusted_merge_threshold = current_merge_threshold + merge_threshold_adjustment

    # 确保参数在合理范围内
    adjusted_gamma = max(0, min(1, adjusted_gamma))
    adjusted_merge_threshold = max(0, min(1, adjusted_merge_threshold))

    return adjusted_gamma, adjusted_merge_threshold


# 获取自适应参数
gamma, merge_threshold = adapt_parameters(bm, clusters)

print("Adapted Parameters:")
print("gamma:", gamma)
print("merge_threshold:", merge_threshold)

# gamma = 0.1
# merge_threshold = 0.5

# 当得到的 clusters 长度不发生变化时退出循环
previous_clusters_length = len(clusters)

print("cluster length:", len(clusters))

# 打印的聚类信息
for cluster_index, cluster in enumerate(clusters):
    print(f"Cluster {cluster_index}:", "Faces:", len(cluster[0]), "Sharp Edges:", len(cluster[1]))

while True:
    edge_section_edges, perimeter_dict, area_dict, rho_dict = get_clusters_info(clusters)
    new_clusters = merge_clusters(clusters, edge_section_edges, perimeter_dict, area_dict, rho_dict, gamma,
                                  merge_threshold)

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

for cluster_faces, _ in clusters:
    data.append(list(cluster_faces))

# Export the data to a JSON file
output_file_path = "export/clusters_Geometry.json"
with open(output_file_path, "w") as json_file:
    json.dump(data, json_file)

bm.free()