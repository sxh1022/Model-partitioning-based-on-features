# 种子面片生长，将特征线形成的封闭区域生成1个聚类

import bpy
import bmesh
import math
import random
import json

# 获取活动对象
obj = bpy.context.active_object

# 获取网格数据
mesh = obj.data

mesh.materials.clear()

# 创建一个BMesh实例
bm = bmesh.new()
bm.from_mesh(mesh)
bm.edges.ensure_lookup_table()
bm.faces.ensure_lookup_table()
bm.verts.ensure_lookup_table()

sharp_edge_indices = [edge.index for edge in bm.edges if edge.select]

# 存储锐边的两个顶点索引的列表
sharp_edges = []

# 遍历锐边索引，获取每条边的两个顶点索引并存储
for edge_index in sharp_edge_indices:
    edge = bm.edges[edge_index]
    vertex_index1, vertex_index2 = edge.verts[0].index, edge.verts[1].index
    sharp_edges.append((vertex_index1, vertex_index2))

bpy.ops.object.mode_set(mode='OBJECT')


def find_connected_faces(start_face, visited_faces, sharp_edges, bm):
    """Find all faces connected to the start face using sharp edges as boundaries"""
    stack = [start_face]
    connected_faces = set()

    while stack:
        current_face = stack.pop()
        visited_faces.add(current_face.index)
        connected_faces.add(current_face.index)

        for edge in current_face.edges:  # Iterate through edges of a face
            vertex_index1, vertex_index2 = edge.verts[0].index, edge.verts[1].index
            if (vertex_index1, vertex_index2) in sharp_edges or (vertex_index2, vertex_index1) in sharp_edges:
                continue

            for neighbor_face in edge.link_faces:
                if neighbor_face.index not in visited_faces:
                    stack.append(neighbor_face)

    return connected_faces


def create_clusters(bm, sharp_edge_indices):
    """Create clusters of faces enclosed by sharp edges"""
    visited_faces = set()
    clusters = []

    for face in bm.faces:
        if face.index in visited_faces:
            continue

        if len(face.verts) != 3:
            continue

        connected_faces = find_connected_faces(face, visited_faces, sharp_edge_indices, bm)

        # Find the sharp edges enclosing the cluster
        cluster_sharp_edges = set()
        for face_index in connected_faces:
            face = bm.faces[face_index]
            for edge in face.edges:
                cluster_sharp_edges.add(edge.index)

        clusters.append((connected_faces, cluster_sharp_edges))

    return clusters


# Create clusters of faces enclosed by sharp edges
clusters = create_clusters(bm, sharp_edges)

# export json
data = []

for cluster_index, (cluster_faces, cluster_sharp_edges) in enumerate(clusters):
    print("cluster ", cluster_index, ":", len(cluster_faces))
    cluster_data = {
        "faces": list(cluster_faces),
        "edge": list(cluster_sharp_edges)
    }
    data.append(cluster_data)

output_file_path = "export/clusters_data_origin.json"
with open(output_file_path, "w") as json_file:
    json.dump(data, json_file, indent=4)

print("Data exported to", output_file_path)

bm.free()
print("over")