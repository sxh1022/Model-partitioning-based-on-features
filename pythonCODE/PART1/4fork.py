# 处理特征的分叉

import bpy
import bmesh

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

sharp_edges = [edge for edge in bm.edges if edge.select]

# 辅助函数：找到断点集合
def find_breakpoints(edges, bm):
    breakpoints = set()
    vertex_degrees = {vert: 0 for vert in bm.verts}

    for edge in edges:
        if isinstance(edge, int):
            edge = bm.edges[edge]
        vertex_degrees[edge.verts[0]] += 1
        vertex_degrees[edge.verts[1]] += 1

    for vert, degree in vertex_degrees.items():
        if degree == 1:
            breakpoints.add(vert.index)

    return breakpoints

# 辅助函数：从断点出发遍历边，记录路径和边数
def traverse_path(vertex, path, edge_count, bm):
    edges = [edge for edge in vertex.link_edges if edge in sharp_edges]

    if len(edges) == 0:
        return None, None

    path.append(vertex.index)
    updated_path = path.copy()

    for edge in edges:
        next_vertex = edge.other_vert(vertex)

        if len([e for e in next_vertex.link_edges if e in sharp_edges]) > 2:
            updated_path.append(next_vertex.index)
            return updated_path, edge_count + 1

    for edge in edges:
        next_vertex = edge.other_vert(vertex)

        if next_vertex.index not in path:
            result_path, result_edge_count = traverse_path(next_vertex, updated_path, edge_count + 1, bm)
            if result_path is not None:
                return result_path, result_edge_count

    return path, edge_count + len(edges)

# 辅助函数：计算平均路径长度和平均分支数量
def compute_average_path_length_and_branch_count(breakpoints, bm):
    total_path_length = 0
    total_branch_count = 0

    for breakpoint in breakpoints:
        path, edge_count = traverse_path(bm.verts[breakpoint], [], 0, bm)
        if path is not None and edge_count is not None:
            total_path_length += len(path)
            total_branch_count += 1

    if total_branch_count > 0:
        average_path_length = total_path_length / total_branch_count
        average_branch_count = total_branch_count / len(breakpoints)
        return average_path_length, average_branch_count
    else:
        return 0, 0

# 找到断点
breakpoints = find_breakpoints(sharp_edges, bm)

# 计算平均路径长度和平均分支数量
average_path_length, average_branch_count = compute_average_path_length_and_branch_count(breakpoints, bm)

print("average_path_length:",average_path_length,"   average_branch_count:",average_branch_count)

desired_path_factor = 0.2   # 保留平均路径长度的20%
desired_branch_factor = 0.2  # 保留平均分支数量的20%

# 根据平均路径长度和平均分支数量计算一个合适的阈值
fork_threshold = (average_path_length * desired_path_factor) + (average_branch_count * desired_branch_factor)
print("fork_threshold:",fork_threshold)

edges_to_deselect = set()  # 存储要取消选择的边

# 遍历断点并处理路径
for breakpoint in breakpoints:
    path, edge_count = traverse_path(bm.verts[breakpoint], [], 0, bm)
    if path is not None and edge_count is not None and edge_count < fork_threshold:
        # 添加要取消选择的边到集合中
        for i in range(len(path) - 1):
            for edge in bm.verts[path[i]].link_edges:
                if bm.verts[path[i+1]] in edge.verts:
                    edges_to_deselect.add(edge)

# 切换到对象模式进行选择
bpy.ops.object.mode_set(mode='OBJECT')

# 取消选择要取消选择的边
for edge in edges_to_deselect:
    edge.select = False

# 更新网格
bm.to_mesh(mesh)
# 切换到编辑模式
bpy.ops.object.mode_set(mode='EDIT')

# 移除分叉边之后的锐边
sharp_edges_after_removal = [edge for edge in bm.edges if edge.select]

# 找到移除分叉边之后的断点
breakpoints_after_removal = find_breakpoints(sharp_edges_after_removal, bm)

print("before remove fork:", len(sharp_edges))
print("after remove fork:", len(sharp_edges_after_removal))
print("breakpoints before remove fork:", len(breakpoints))
print("breakpoints after remove fork:", len(breakpoints_after_removal))
bm.free()
print("over")