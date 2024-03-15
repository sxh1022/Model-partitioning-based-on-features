# 过滤特征中的小连通区域

import bpy
import bmesh
import statistics

# 获取活动对象
obj = bpy.context.active_object
# 获取网格数据
mesh = obj.data

# 创建一个BMesh实例
bm = bmesh.new()
bm.from_mesh(mesh)
bm.edges.ensure_lookup_table()
bm.faces.ensure_lookup_table()

# 存储锐边索引的集合
sharp_edge_indices = {edge.index for edge in bm.edges if edge.select}
print("before filter:", len(sharp_edge_indices))


def find_connected_components(bm, edges):
    connected_components = []  # 存储连通区域的列表，每个区域是一组边的集合
    visited_edges = set()  # 存储已访问的边索引

    while edges:
        stack = [edges.pop()]  # 以新的锚边开始

        visited = set()

        while stack:
            edge_idx = stack.pop()
            visited.add(edge_idx)
            visited_edges.add(edge_idx)
            edge = bm.edges[edge_idx]

            # 通过边的共享顶点找到相邻边，但仅添加未访问的边
            adjacent_edges = set()
            for vert in edge.verts:
                adjacent_edges.update(
                    linked_edge.index
                    for linked_edge in vert.link_edges
                    if (
                            linked_edge.index in edges
                            and linked_edge.index not in visited
                            and linked_edge.index not in visited_edges
                    )
                )

            stack.extend(adjacent_edges)
            edges.discard(edge_idx)  # 从边集合中移除已处理的边

        connected_components.append(visited)

    return connected_components


# 获取连通区域
connected_components = find_connected_components(bm, sharp_edge_indices)

# 收集连通区域中边的数量
component_edge_counts = [len(component) for component in connected_components]

# 计算所有连通区域中边的数量总和
total_edge_count = sum(component_edge_counts)
# print("total_edge_count:",total_edge_count)

# 计算连通区域中边数量的最大值和最小值
max_edge_count = max(component_edge_counts)
min_edge_count = min(component_edge_counts)

# 计算模型的大小（例如使用模型的最大维度）
model_size = max(obj.dimensions)

# 计算最大和最小边数量相对于模型大小的比例
max_edge_ratio = max_edge_count / model_size
min_edge_ratio = min_edge_count / model_size

# 限制阈值的最小和最大值，确保阈值不会过小或过大
min_threshold = min(component_edge_counts)  # 最小阈值
max_threshold = int(statistics.median(list(set(component_edge_counts))))  # 最大阈值

# print("min_threshold :",min_threshold )
# print("max_threshold :",max_threshold)
# 限制最大阈值，确保不会过大
max_threshold = min(max_threshold, len(bm.edges) * 0.05)

# 根据比例因子来调整一个基础阈值，使其落在合适的范围内
base_threshold = 10  # 基础阈值
adjusted_threshold = base_threshold * (max_edge_ratio + min_edge_ratio) / 2

edge_num_threshold = max(min(adjusted_threshold, max_threshold), min_threshold)

# edge_num_threshold =6

print("edge_num_threshold:", edge_num_threshold)

# 过滤掉锐边数量小于阈值的连通区域
filtered_components = [component for component in connected_components if len(component) >= edge_num_threshold]
# print("filtered_components:",filtered_components)

# 收集每个连通区域中的锐边数量，存储在 component_edge_counts 列表中。
filtered_components_counts = [len(component) for component in filtered_components]

## 输出每个连通区域中边的数量
# for index, edge_count in enumerate(filtered_components_counts):
#    print(f"Connected Component {index + 1} has {edge_count} sharp edges.")

# 切换到对象模式进行选择
bpy.ops.object.mode_set(mode='OBJECT')

# 重置之前的选择状态
for edge in bm.edges:
    edge.select = False
for face in bm.faces:
    face.select = False

# 设置新的选择状态
for component in filtered_components:
    for edge_idx in component:
        bm.edges[edge_idx].select = True

print("after filter:", len([edge for edge in bm.edges if edge.select]))
print("area num:", len(filtered_components))
# total_filtered_edge_count = sum(filtered_components_counts)
# print("Total Edge Count:", total_filtered_edge_count)

# 更新网格
bm.to_mesh(mesh)
# 切换到编辑模式
bpy.ops.object.mode_set(mode='EDIT')
bm.free()
print("over")
