# 根据权重进行初始简单合并，合并掉一些面积小的块

import bpy
import math
import random
import json
from mathutils import Vector
import statistics
import numpy as np
import json
import heapq  # 用于优先队列的模块

# 获取活动对象
obj = bpy.context.active_object

# 获取网格数据
mesh = obj.data


# 聚类之间的距离函数，这里使用欧几里得距离
def distance(cluster1, cluster2, mesh):
    centroid1 = calculate_centroid(cluster1, mesh)
    centroid2 = calculate_centroid(cluster2, mesh)
    return math.sqrt((centroid1[0] - centroid2[0]) ** 2 + (centroid1[1] - centroid2[1]) ** 2)


# 计算聚类的平均法线方向
def calculate_cluster_normal(cluster, mesh):
    normals = []
    for face_index in cluster[0]:
        normals.append(mesh.polygons[face_index].normal)
    return sum(normals, Vector()) / len(normals)


# 计算面的中心点
def calculate_face_center(mesh, face_index):
    face = mesh.polygons[face_index]
    total_coord = Vector((0, 0, 0))
    for vertex_index in face.vertices:
        total_coord += mesh.vertices[vertex_index].co
    center = total_coord / len(face.vertices)
    return center


# 计算聚类的中心点
def calculate_centroid(cluster, mesh):
    total_x = 0
    total_y = 0
    for face_index in cluster[0]:
        center = calculate_face_center(mesh, face_index)
        total_x += center[0]
        total_y += center[1]
    centroid_x = total_x / len(cluster[0])
    centroid_y = total_y / len(cluster[0])
    return (centroid_x, centroid_y)


# 合并距离最近的聚类  每次都合并面积最小的聚类，clusters[0]
def merge_clusters1(clusters, mesh):
    max_score = -float('inf')
    merge_indices = (-1, -1)
    distance_weight = 0.4
    shared_edges_weight = 0.6
    normal_similarity_weight = 0.8  # 新的法线方向相似性权重

    for i in range(len(clusters)):
        print("i:", i)
        for j in range(i + 1, len(clusters)):
            cluster1_faces, cluster1_edges = clusters[i]
            cluster2_faces, cluster2_edges = clusters[j]

            # 在判断是否合并两个聚类时，检查它们之间是否存在共享的边
            common_edges = cluster1_edges & cluster2_edges
            num_edges = len(common_edges)
            if num_edges > 0:
                # 获取共享边的权重信息
                total_weight = 0.0

                flag = False

                for edge_index in common_edges:
                    if edge_index in edge_weights:
                        edge_weight = edge_weights[edge_index]
                        if (edge_weight > 0.8):
                            flag = True
                            break
                        total_weight += edge_weight
                print("total_weight:", total_weight)
                if (flag == True):
                    continue
                #                average_weight = total_weight / num_edges
                #                print("average_weight:",average_weight)
                #                # 如果共享边中存在权重为1的边，则不进行合并
                #                if average_weight==1.0:
                #                    continue

                dist = distance(clusters[i], clusters[j], mesh)
                shared_edges_count = len(common_edges)

                # 计算综合合并得分，根据需要调整权重
                merge_score = distance_weight * (1 / dist) + shared_edges_weight * shared_edges_count
                #                merge_score = distance_weight * (1 / dist) + shared_edges_weight * (1/total_weight)

                # 计算法线方向相似性得分
                normal_similarity = 1.0 - calculate_cluster_normal(clusters[i], mesh).dot(
                    calculate_cluster_normal(clusters[j], mesh))
                merge_score += normal_similarity_weight * normal_similarity

                if merge_score > max_score:
                    max_score = merge_score
                    merge_indices = (i, j)

    if merge_indices != (-1, -1):
        cluster1, cluster2 = clusters[merge_indices[0]], clusters[merge_indices[1]]
        merged_faces = cluster1[0] | cluster2[0]
        merged_sharp_edges = cluster1[1] | cluster2[1]

        new_cluster = (merged_faces, merged_sharp_edges)
        clusters.pop(max(merge_indices))
        clusters.pop(min(merge_indices))
        clusters.append(new_cluster)
        return True
    return False


# 优先与共享边更多的聚类进行合并
def merge_clusters(clusters, mesh):
    #    print("merge")
    min_weights = float('inf')
    merge_indices = (-1, -1)

    for i in range(1, len(clusters)):
        cluster1_faces, cluster1_edges = clusters[0]
        cluster2_faces, cluster2_edges = clusters[i]

        # 在判断是否合并两个聚类时，检查它们之间是否存在共享的边
        common_edges = cluster1_edges & cluster2_edges
        num_edges = len(common_edges)
        if num_edges > 0:
            # 获取共享边的权重信息
            total_weight = 0.0

            for edge_index in common_edges:
                if edge_index in edge_weights:
                    edge_weight = edge_weights[edge_index]
                    total_weight += edge_weight

            average_weight = total_weight / num_edges

            if min_weights > average_weight:
                min_weights = average_weight
                merge_indices = (0, i)

    if merge_indices != (-1, -1):
        cluster1, cluster2 = clusters[merge_indices[0]], clusters[merge_indices[1]]
        merged_faces = cluster1[0] | cluster2[0]
        merged_sharp_edges = cluster1[1] | cluster2[1]
        new_cluster = (merged_faces, merged_sharp_edges)
        clusters.pop(max(merge_indices))
        clusters.pop(min(merge_indices))
        clusters.append(new_cluster)
        return True
    return False


# 文件路径
input_file_path = "export/clusters_data_origin.json"

# 读取JSON文件
with open(input_file_path, "r") as json_file:
    json_data = json.load(json_file)

# 重构clusters
clusters = []
for cluster_data in json_data:
    cluster_faces = set(cluster_data["faces"])
    cluster_sharp_edges = set(cluster_data["edge"])
    clusters.append((cluster_faces, cluster_sharp_edges))

print("read file")
print("before:")
print("cluster num:", len(clusters))
# 输出重构的clusters信息
for cluster_index, (cluster_faces, cluster_sharp_edges) in enumerate(clusters):
    print(f"Cluster {cluster_index}:", ",Faces:", len(cluster_faces), ",Enclosing Sharp Edges:",
          len(cluster_sharp_edges))

# 从 JSON 文件读取边的权重信息到新的字典中
input_weights_file_path = "export/edge_weights.json"
with open(input_weights_file_path, "r") as json_file:
    loaded_edge_weights = json.load(json_file)

# 创建一个新的字典来存储权重信息
edge_weights = {}

# 将读取的数据存储到新的字典中
for edge_index_str, weight in loaded_edge_weights.items():
    edge_index = int(edge_index_str)
    edge_weights[edge_index] = weight

# print("edge_weights:",edge_weights)
print("==================================================================")

# 设置阈值，小于该阈值面数的聚类将被认为是小的

# 统计所有聚类中的面片数量
all_face_counts = [len(cluster[0]) for cluster in clusters]

# 将面片数量排序
sorted_face_counts = sorted(all_face_counts)
# area_threshold = int(statistics.mean(sorted_face_counts) * 0.9)

# 选择百分位数阈值的范围（例如 5% 到 20%）
min_percentile = 5
max_percentile = 20

# 计算阈值的初始候选值，选择在上述范围内的中位数
initial_threshold = np.percentile(sorted_face_counts, (min_percentile + max_percentile) / 2)

# 考虑模型整体大小，根据需要进行适当调整
model_size = len(mesh.polygons)
area_threshold = initial_threshold * (model_size / 1000)  # 假设模型大小为1000个面

# 考虑聚类数量，根据需要进行适当调整
num_clusters = len(clusters)
area_threshold *= (1 + num_clusters * 0.05)  # 假设每个聚类增加5%的阈值

print("area_threshold:", area_threshold)

## 设置需要合并的次数，或者根据其他条件停止合并
# merge_count = 5

## 逐步合并聚类，直到达到预定的聚类数量
# while merge_count > 0:
#    clusters.sort(key=lambda cluster: len(cluster[0]))  # 按照面片数量从小到大排序
#    if not merge_clusters(clusters, mesh):
#        break
#    merge_count -= 1

# 逐步合并聚类，直到没有面数小于阈值的聚类
while any(len(cluster[0]) < area_threshold for cluster in clusters):
    clusters.sort(key=lambda cluster: len(cluster[0]))  # 按照面片数量从小到大排序
    if not merge_clusters(clusters, mesh):
        break

print("after:")
print("cluster num:", len(clusters))

# export json
data = []

for cluster_index, (cluster_faces, cluster_sharp_edges) in enumerate(clusters):
    # 打印合并后的聚类信息
    print(f"Cluster {cluster_index}:", ",Faces:", len(cluster_faces), ",Enclosing Sharp Edges:",
          len(cluster_sharp_edges))
    cluster_data = {
        "faces": list(cluster_faces),
        "edge": list(cluster_sharp_edges)
    }
    data.append(cluster_data)

output_file_path = "export/clusters_data1.json"
with open(output_file_path, "w") as json_file:
    json.dump(data, json_file, indent=4)

print("Data exported to", output_file_path)

print("over")