# 对特征线进行延伸，封闭特征线，不创建新的边

import bpy
import bmesh
import statistics
import mathutils
import json

# 获取活动对象
obj = bpy.context.active_object
# 获取网格数据
mesh = obj.data

# 创建一个BMesh实例
bm = bmesh.new()
bm.from_mesh(mesh)
bm.edges.ensure_lookup_table()
bm.faces.ensure_lookup_table()


# ======================================计算每个连通区域到边缘的距离=================================================

# 计算点到最近边界边的距离
def distance_to_boundary(vert, boundary_edges):
    min_distance = float('inf')
    for edge in boundary_edges:
        for v in edge.verts:
            distance = (vert.co - v.co).length
            if distance < min_distance:
                min_distance = distance
    return min_distance


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


# 获取已选锐边
selected_edges = [edge for edge in bm.edges if edge.select]
selected_edges_index = {edge.index for edge in selected_edges}

selected_vertices = set()
# 收集已选锐边的顶点
for edge in selected_edges:
    selected_vertices.update(edge.verts)

# 收集模型边界的顶点
boundary_edges = [edge for edge in bm.edges if edge.is_boundary]
boundary_vertices = set()
for edge in boundary_edges:
    boundary_vertices.update(edge.verts)

# 获取连通区域
connected_components = find_connected_components(bm, selected_edges_index)

# 创建一个字典来存储连通区域的信息
connected_components_info = []

# 计算每个连通区域到边缘的距离
for index, component in enumerate(connected_components):
    distance = 0.0
    vertex_set = set()
    component_edges = []

    for edge_idx in component:
        component_edges.append(edge_idx)
        for vertex in bm.edges[edge_idx].verts:
            vertex_set.add(vertex)

    for vert in vertex_set:
        distance += distance_to_boundary(vert, boundary_edges)
    distance /= len(component)

    component_info = {
        "index": index + 1,
        "edges": component_edges,
        "distance": distance,
        "weight": 0.0  # 临时设置为0，稍后会更新权重
    }

    connected_components_info.append(component_info)

# 计算权重
max_distance = max(info['distance'] for info in connected_components_info)
min_distance = min(info['distance'] for info in connected_components_info)

# 计算初始权重时，考虑距离和边的数量
for info in connected_components_info:
    if max_distance != min_distance:
        weight = (info['distance'] - min_distance) / (max_distance - min_distance)
    else:
        # 如果最大距离和最小距离相等，默认权重0
        weight = 0.0
    info['weight'] = weight

# 输出每个连通区域的边的数量、距离和权重
for info in connected_components_info:
    print(f"index: {info['index']} len: {len(info['edges'])} distance: {info['distance']} weight: {info['weight']}")


# 定义一个函数来寻找每个连通区域的断点
def find_breakpoints_for_component(bm, component_edges):
    breakpoints = set()
    vert_edge_counts = {}

    for edge_idx in component_edges:
        edge = bm.edges[edge_idx]
        for vert in edge.verts:
            vert_edge_counts[vert] = vert_edge_counts.get(vert, 0) + 1

    for vert, count in vert_edge_counts.items():
        if count == 1:
            breakpoints.add(vert)

    return breakpoints


# 定义一个函数，用于计算两个向量之间的角度
def angle_between_vectors(v1, v2):
    if v1.length > 0 and v2.length > 0:
        return v1.angle(v2)
    return 0.0


# 延伸特征线，从断点出发找到方向相近的下一个点
def extend_edges(breakpoint, selected_edges, boundary_vertices):
    adjacent_edges = [edge for edge in breakpoint.link_edges if edge.index in selected_edges]
    if len(adjacent_edges) == 1:
        selected_vertex = [v for v in adjacent_edges[0].verts if v != breakpoint][0]
        reference_vector = breakpoint.co - selected_vertex.co

        # 获取断点的法线方向
        breakpoint_normal = breakpoint.normal

        closest_vertex = None
        closest_angle = float('inf')

        # 找到最接近的顶点
        for edge in breakpoint.link_edges:
            for vert in edge.verts:
                if vert != breakpoint and vert != selected_vertex:
                    vector = vert.co - breakpoint.co

                    # 计算向量在法线方向上的投影
                    projected_vector = vector - vector.dot(breakpoint_normal) * breakpoint_normal
                    angle = angle_between_vectors(projected_vector, reference_vector)

                    if angle < closest_angle:
                        closest_angle = angle
                        closest_vertex = vert

        if closest_vertex is not None:
            # 将 closest_vertex 和 breakpoint 之间的边设置为选中状态
            for edge in closest_vertex.link_edges:
                if breakpoint in edge.verts:
                    edge.select = True
                    selected_edges.append(edge.index)
            return closest_vertex
        return None
    return None


# 切换到对象模式
bpy.ops.object.mode_set(mode='OBJECT')

# 创建一个字典来存储边的权重
edge_weights = {edge.index: 0.0 for edge in bm.edges}

# 对每个连通区域进行断点查找和延伸
for component_info in connected_components_info:
    component_edges = component_info["edges"]
    component_weight = component_info["weight"]
    for edge_index in component_edges:
        # 检查当前权重与新权重的大小，选择较大的值
        edge_weights[edge_index] = max(edge_weights.get(edge_index, 0.0), component_weight)

    # 找到该连通区域的断点
    breakpoints = find_breakpoints_for_component(bm, component_edges)
    # 延伸出的边的权重进行衰减
    #    weight = component_weight * 0.8

    # 遍历所有断点
    while breakpoints:
        new_breakpoints = set()
        for breakpoint in breakpoints:

            extend_points = extend_edges(breakpoint, component_edges, boundary_vertices)
            if extend_points is not None:
                if extend_points in boundary_vertices:
                    continue
                else:
                    # 更新延伸出的边的权重，将其设置为连通区域的权重，保留较大的权重
                    for edge_index in component_edges:
                        if edge_index not in edge_weights:
                            edge_weights[edge_index] = component_weight
                        else:
                            edge_weights[edge_index] = max(edge_weights[edge_index], component_weight)
                    new_breakpoints.add(extend_points)
        breakpoints = new_breakpoints

# print("edge_weights:",edge_weights)
# 更新网格
bm.to_mesh(mesh)
# 切换到编辑模式
bpy.ops.object.mode_set(mode='EDIT')

# 导出边的权重信息到 JSON 文件
output_weights_file_path = "export/edge_weights.json"
with open(output_weights_file_path, "w") as json_file:
    json.dump(edge_weights, json_file, indent=4)

print("Edge weights exported to", output_weights_file_path)

bm.free()
print("over")