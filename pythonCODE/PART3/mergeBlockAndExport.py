import bpy
import random
import bmesh
import json
import mathutils
import numpy as np

# 获取活动对象
obj = bpy.context.active_object

# 获取网格数据
mesh = obj.data

# 创建一个 BMesh 实例以便更容易地操作三角网格数据
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

# 全局变量，定义垂直和水平方向
# vertical_direction = mathutils.Vector((0, -1, 0))
# horizontal_direction = mathutils.Vector((1, 0, 0))

vertical_direction = up_vector
horizontal_direction = right_vector


# 两种不同面片分块方法的面片索引
# 读取JSON文件
def load_indices_from_json(file_path):
    with open(file_path, "r") as json_file:
        return json.load(json_file)


method1_indices = load_indices_from_json("export/clusters_Geometry.json")
method2_indices = load_indices_from_json("export/clusters_Avg.json")
# method2_indices = load_indices_from_json("export/clusters_Curvature.json")

print("method1_indices:", len(method1_indices))
print("method2_indices:", len(method2_indices))


# 拆分不连通的面片
def split_connected_faces(bm, faces):
    connected_components = []  # 存储连通区域的列表
    visited_faces = set()  # 存储已访问的面片索引
    faces = set(faces)

    while faces:
        stack = [faces.pop()]  # 以新的起始面片开始

        visited = set()

        while stack:
            face_idx = stack.pop()
            visited.add(face_idx)
            visited_faces.add(face_idx)
            face = bm.faces[face_idx]

            # 通过共享的顶点找到相邻面片，但仅添加未访问的面片
            adjacent_faces = set()
            for vert in face.verts:
                adjacent_faces.update(
                    linked_face.index
                    for linked_face in vert.link_faces
                    if (
                            linked_face.index in faces
                            and linked_face.index not in visited
                            and linked_face.index not in visited_faces
                    )
                )

            stack.extend(adjacent_faces)
            faces.discard(face_idx)  # 从面片集合中移除已处理的面片

        connected_components.append(visited)

    return connected_components


# 组合两种分块方法得到最终的分块结果
def combine_block_indices(bm, method1_indices, method2_indices):
    block_info = {}  # 创建一个字典，用于存储面片所属的块信息
    final_indices = []  # 存储最终的分块结果

    # 遍历第一种分块方法的块
    for block1_index, block1_faces in enumerate(method1_indices):
        for face_index in block1_faces:
            if face_index not in block_info:
                block_info[face_index] = set()
            block_info[face_index].add(block1_index)

    def check_and_split_block(block2_index, block2_faces):
        new_blocks = []

        # 遍历第一种分块方法的块，检查是否需要拆分
        for block1_index, block1_faces in enumerate(method1_indices):
            common_faces = set(block2_faces).intersection(block1_faces)
            if common_faces:
                non_common_faces = set(block2_faces).difference(block1_faces)
                new_blocks.append(list(common_faces))
                block2_faces = list(non_common_faces)

        new_blocks.append(list(block2_faces))
        return new_blocks

    # 遍历第二种分块方法的块
    for block2_index, block2_faces in enumerate(method2_indices):
        split_blocks = check_and_split_block(block2_index, block2_faces)
        final_indices.extend(split_blocks)

    final_indices = [block for block in final_indices if block]  # 移除空块

    print("first_final_indices:", len(final_indices))

    def split_blocks(bm, final_indices):
        new_final_indices = []

        # 拆分不连通的面片
        for block_faces in final_indices:
            split_blocks = split_connected_faces(bm, block_faces)
            new_final_indices.extend(split_blocks)

        return new_final_indices

    new_final_indices = split_blocks(bm, final_indices)

    face_to_block_mapping = {}  # 将面片索引映射到块索引的字典
    combined_method1_indices = []  # 存储与method1_indices的组合
    combined_method2_indices = []  # 存储与method2_indices的组合
    final_block_indices = set(range(len(new_final_indices)))  # 包含所有块索引的集合

    for block_index, block in enumerate(new_final_indices):
        for face_index in block:
            face_to_block_mapping[face_index] = block_index

    # 遍历method1_indices
    for method1_block in method1_indices:
        combined_indices = set()  # 使用集合存储块的索引，以去除重复值
        for face_index in method1_block:
            if face_index in face_to_block_mapping:
                combined_indices.add(face_to_block_mapping[face_index])
                final_block_indices.discard(face_to_block_mapping[face_index])  # 移除块索引
        combined_method1_indices.append(list(combined_indices))

    # 遍历method2_indices
    for method2_block in method2_indices:
        combined_indices = set()  # 使用集合存储块的索引，以去除重复值
        for face_index in method2_block:
            if face_index in face_to_block_mapping:
                combined_indices.add(face_to_block_mapping[face_index])
                final_block_indices.discard(face_to_block_mapping[face_index])  # 移除块索引
        combined_method2_indices.append(list(combined_indices))

    # 处理final_indices中剩余的块，它们在两种方法中都没有找到对应的块
    for _ in final_block_indices:
        combined_method1_indices.append([])
        combined_method2_indices.append([])

    return new_final_indices, combined_method1_indices, combined_method2_indices


# 组合两种分块方法得到最终的分块结果
final_indices, combined_method1_indices, combined_method2_indices = combine_block_indices(bm, method1_indices,
                                                                                          method2_indices)
print("final_indices:", len(final_indices))
# 输出排序后的分块结果
for i, block in enumerate(final_indices):
    print(f"Block {i}: {len(block)}")
total_face_count = sum(len(block) for block in final_indices)
print("total_face_count:", total_face_count)
print("combined_method1_indices:", combined_method1_indices)
print("combined_method2_indices:", combined_method2_indices)


# =========================================================================
# 计算给定面片索引列表的左上角坐标
def calculate_top_left_corner(cluster_indices, mesh):
    if not cluster_indices:
        return None

    # 初始化最左上角点的坐标
    top_left_corner = None

    # 遍历每个块中的面片索引
    for face_index in cluster_indices:
        face = mesh.polygons[face_index]

        for vert_index in face.vertices:
            vertex = mesh.vertices[vert_index]
            vertex_co = vertex.co

            # 计算点的投影在垂直和水平方向上的分量
            vertical_component = vertex_co.dot(vertical_direction)
            horizontal_component = vertex_co.dot(horizontal_direction)

            # 更新最左上角点
            if top_left_corner is None or vertical_component > top_left_corner[1] or (
                    vertical_component == top_left_corner[1] and horizontal_component < top_left_corner[0]):
                top_left_corner = (horizontal_component, vertical_component)

    return mathutils.Vector((top_left_corner[0], top_left_corner[1], 0))


# 按照指定方向对块索引进行排序
def sort_blocks_by_position(top_left_corners_method, block_indices, return_tuples=True):
    # 创建一个排序函数，首先按照上下方向排序，然后按照从左到右排序
    def custom_sort(block_index):
        top_left_corner = mathutils.Vector(top_left_corners_method[block_index])
        return (-top_left_corner.dot(vertical_direction), top_left_corner.dot(horizontal_direction))

    # 使用排序函数对块索引进行排序
    sorted_block_indices = sorted(enumerate(block_indices), key=lambda x: custom_sort(x[0]))

    if return_tuples:
        # 提取排序后的块索引和面片索引
        sorted_indices = [(index, indices) for index, indices in sorted_block_indices]
    else:
        # 只返回排序后的索引列表
        sorted_indices = [indices for _, indices in sorted_block_indices]

    return sorted_indices


# 计算多个块的左上角坐标并排序块索引
def calculate_top_left_corners_for_method(method_indices, final_indices, mesh):
    top_left_corners = {}
    sorted_method_indices = []

    for i, cluster_indices in enumerate(method_indices):
        top_left_corner = calculate_top_left_corner(cluster_indices, mesh)
        top_left_corners[i] = top_left_corner

    for i, cluster_indices in enumerate(method_indices):
        sorted_method_indices.append(cluster_indices)

    # 按照自定义排序函数对块索引进行排序
    sorted_method_indices = sort_blocks_by_position(top_left_corners, sorted_method_indices)
    sorted_method_indices = [cluster_indices for cluster_indices, _ in sorted_method_indices]

    return top_left_corners, sorted_method_indices


# 重新排序组合块索引
def reorder_combined_indices(sorted_indices, combined_indices):
    reordered_combined_indices = [combined_indices[i] for i in sorted_indices]
    return reordered_combined_indices


# 计算给定子列表的左上角坐标并排序
def calculate_left_top_corners_for_sublist(sublist, final_indices, mesh):
    left_top_corners = []
    for block_index in sublist:
        top_left_corner = calculate_top_left_corner(final_indices[block_index], mesh)
        left_top_corners.append(top_left_corner)
    return left_top_corners


# 获取最终的块索引顺序
def getFinalIndices(final_indices, method_indices, combined_indices):
    top_left_corners, sorted_method_indices = calculate_top_left_corners_for_method(method_indices, final_indices, mesh)
    reordered_combined_indices = reorder_combined_indices(sorted_method_indices, combined_indices)

    for i, sublist in enumerate(reordered_combined_indices):
        left_top_corners = calculate_left_top_corners_for_sublist(sublist, final_indices, mesh)
        sorted_sublist = sort_blocks_by_position(left_top_corners, sublist, False)
        reordered_combined_indices[i] = sorted_sublist

    flattened_combined_indices = [item for sublist in reordered_combined_indices for item in sublist]
    final_indices = [final_indices[i] for i in flattened_combined_indices]
    return final_indices


# 根据权重评估排序并返回最终结果
def evaluate_sorting(final_indices, method1_indices, combined_method1_indices, method2_indices,
                     combined_method2_indices, weight_Geometry, weight_Average):
    if weight_Geometry > weight_Average:
        return getFinalIndices(final_indices, method1_indices, combined_method1_indices)
    else:
        return getFinalIndices(final_indices, method2_indices, combined_method2_indices)


# 设置权重，以权衡两种排序方式
weight_Geometry = 0.4
weight_Average = 0.6

# 使用评价函数评估两种排序方式
final_indices = evaluate_sorting(final_indices, method1_indices, combined_method1_indices, method2_indices,
                                 combined_method2_indices, weight_Geometry, weight_Average)


# ===========================================================================================================
def merge_small_blocks(final_indices, combined_method1_indices, combined_method2_indices, longest_edge_index,
                       arm_length):
    # 创建一个集合，用于跟踪已合并的小块索引
    merged_small_blocks = set()

    # 创建一个字典，用于存储面片所属的块信息
    block_info = {}

    # 遍历final_indices中的块，建立面片到块的映射
    for block_index, block_faces in enumerate(final_indices):
        for face_index in block_faces:
            block_info[face_index] = block_index

    # 找到最大块的数量
    max_block_size = max(len(block) for block in final_indices)
    print("max_block_size:", max_block_size)

    # 计算小块的阈值，取最大块数量的5%
    block_size_threshold = max_block_size * 0.05
    print("block_size_threshold:", block_size_threshold)

    # 用于判断是否可以合并的函数
    def can_merge_blocks(block1, block2):
        # 计算合并后的面片集合
        merged_block = set(block1) | set(block2)

        # 计算当前块的长度
        min_coord_on_longest_edge = float('inf')
        max_coord_on_longest_edge = float('-inf')

        # 遍历块中的面片，找到块在最长边上的最小和最大坐标,投影
        for face_index in merged_block:
            face = bm.faces[face_index]
            for vert in face.verts:
                coord_on_longest_edge = vert.co.dot(longest_edge_direction)
                min_coord_on_longest_edge = min(min_coord_on_longest_edge, coord_on_longest_edge)
                max_coord_on_longest_edge = max(max_coord_on_longest_edge, coord_on_longest_edge)

        block_length_on_longest_edge = max_coord_on_longest_edge - min_coord_on_longest_edge

        # 判断是否可以合并，最长边小于arm_length时才合并
        return block_length_on_longest_edge < arm_length

    # 存储小块的索引
    small_blocks = [block for block in final_indices if len(block) < block_size_threshold]
    small_block_indices = [i for i, block in enumerate(final_indices) if len(block) < block_size_threshold]

    small_blocks_and_indices = list(zip(small_block_indices, small_blocks))
    print("small_blocks_and_indices:", small_blocks_and_indices)

    # 定义一个函数，用于获取 adjacent_block_info 中的共享边缘数
    def get_shared_edges(adjacent_block_info, block_index):
        for adj_block_index, shared_edges in adjacent_block_info:
            if adj_block_index == block_index:
                return shared_edges
        return 0

    for small_block_index, small_block in small_blocks_and_indices:
        if small_block_index in merged_small_blocks:
            # 如果小块已经被合并，跳过
            continue
        print("index:", small_block_index)
        merged = False
        adjacent_block_info = []  # 用于存储相邻块的信息，包括索引和共享的边缘数

        # 遍历小块的面片，找到相邻块的索引并计算共享的边缘数
        for face_index in small_block:
            shared_edges = 0  # 初始化共享的边缘数
            # 查找与当前块中的面相邻的块索引
            for vertex_index in bm.faces[face_index].verts:
                for adjacent_face in vertex_index.link_faces:
                    if adjacent_face.index != face_index:
                        adjacent_block_index = block_info.get(adjacent_face.index)
                        if adjacent_block_index is not None:
                            shared_edges += 1  # 增加共享的边缘数
                            adjacent_block_info.append((adjacent_block_index, shared_edges))

        # 移除当前块自身的索引
        adjacent_block_info = [(adj_block_index, shared_edges) for adj_block_index, shared_edges in adjacent_block_info
                               if adj_block_index != block_info[face_index]]

        # 找到在两种方法中包含 small_block_index 的其他 index
        matching_method1_sublist = [sublist for sublist in combined_method1_indices if small_block_index in sublist]
        same_method1_indices = [index for index in matching_method1_sublist[0] if index != small_block_index]

        # 按照 adjacent_block_info 中的共享边缘数对索引进行排序
        sorted_same_method1_indices = sorted(same_method1_indices,
                                             key=lambda index: get_shared_edges(adjacent_block_info, index),
                                             reverse=True)
        print("same_method1_indices (sorted):", sorted_same_method1_indices)

        merge_index = -1

        # 尝试与相邻块中的method1_indices中的块合并
        for adjacent_index in sorted_same_method1_indices:
            if can_merge_blocks(final_indices[adjacent_index], small_block):
                # 合并块
                final_indices[adjacent_index].update(small_block)
                for face_index in small_block:
                    block_info[face_index] = adjacent_index  # 更新块信息
                print(small_block_index, " and ", adjacent_index, " merge")
                merge_index = adjacent_index
                merged = True

        # 如果没有与method1_indices中的块合并成功，尝试与method2_indices中共享边数量最大的块合并
        if not merged:
            print("method2")

            # 找到在两种方法中包含small_block_index的其他 index
            matching_method2_sublist = [sublist for sublist in combined_method2_indices if small_block_index in sublist]
            # 从匹配的子列表中找到边数最大的索引
            same_method2_indices = matching_method2_sublist[0]
            max_shared_edges_index = max(same_method2_indices,
                                         key=lambda index: get_shared_edges(adjacent_block_info, index))
            print("Max shared edges index (method2):", max_shared_edges_index)

            # 合并块
            final_indices[max_shared_edges_index].update(small_block)
            for face_index in small_block:
                block_info[face_index] = max_shared_edges_index  # 更新块信息
            print(small_block_index, " and ", max_shared_edges_index, " merge")
            merge_index = max_shared_edges_index
            merged = True
        else:
            print("method1")

        # 如果合并成功，将小块索引添加到已合并的集合中
        if merged:
            merged_small_blocks.add(small_block_index)
            if merge_index in small_block_indices:
                merged_small_blocks.add(merge_index)

    # 删除已经合并的小块
    final_indices = [block for i, block in enumerate(final_indices) if i not in merged_small_blocks]

    return final_indices


# 合并小块并指定 arm_length=1
final_indices = merge_small_blocks(final_indices, combined_method1_indices, combined_method2_indices,
                                   longest_edge_index, arm_length=1)
print("merge:", len(final_indices))

# 输出排序后的分块结果
for i, block in enumerate(final_indices):
    print(f"Block {i}: {len(block)}")
total_face_count = sum(len(block) for block in final_indices)
print("total_face_count:", total_face_count)


def assign_materials(mesh, indices):
    """为每个分块分配材质"""
    mesh.materials.clear()

    for i in range(len(indices)):
        material = bpy.data.materials.new(f"mat_{i}")
        material.diffuse_color = (random.random(), random.random(), random.random(), 1.0)
        mesh.materials.append(material)

    face_cluster_indices = [-1] * len(mesh.polygons)
    for i, cluster_indices in enumerate(indices):
        for index in cluster_indices:
            face_cluster_indices[index] = i

    for i, material_index in enumerate(face_cluster_indices):
        mesh.polygons[i].material_index = material_index


# 调用 assign_materials 函数
assign_materials(mesh, final_indices)

# ====================================================================================================
# 获取边缘
print("boundary++final_indices: ", len(final_indices))
boundary_edges_in_partitions = [[] for _ in range(len(final_indices))]
for edge in bm.edges:
    adjacent_faces = edge.link_faces
    # 判断边缘所属分区
    if len(adjacent_faces) == 1:
        for i, partition in enumerate(final_indices):
            if adjacent_faces[0].index in partition:
                boundary_edges_in_partitions[i].append(edge.index)
    elif len(adjacent_faces) == 2:
        face1_partition = None
        face2_partition = None
        for i, partition in enumerate(final_indices):
            if adjacent_faces[0].index in partition:
                face1_partition = i
            if adjacent_faces[1].index in partition:
                face2_partition = i
        if face1_partition is not None and face2_partition is not None and face1_partition != face2_partition:
            boundary_edges_in_partitions[face1_partition].append(edge.index)
            boundary_edges_in_partitions[face2_partition].append(edge.index)

# 输出每个分区中的边缘数量
for i, edges_in_partition in enumerate(boundary_edges_in_partitions):
    print(f"Partition {i}: {len(edges_in_partition)} edges")

# 创建一个字典，用于存储每个分区的数据
clusters = {}

# 存储 final_indices 和 boundary_edges_in_partitions
for i, (cluster_indices, boundary_edges) in enumerate(zip(final_indices, boundary_edges_in_partitions)):
    clusters[i] = {
        'faces': cluster_indices,
        'edge': boundary_edges
    }


# ====================================================================================================
# 对边缘进行处理

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


def separate_largest_connected_component(connected_components):
    # 找到最大连通区域
    largest_component = max(connected_components, key=len)

    # 从连通区域列表中移除最大连通区域
    connected_components.remove(largest_component)

    return largest_component


boundary_edges = set(edge.index for edge in bm.edges if edge.is_boundary)
print("boundary_edges:", len(boundary_edges))
connected_components = find_connected_components(bm, boundary_edges)

print("connected_components:", len(connected_components))

# 找到最大连通区域并分离,将最大连通区域看作是外边缘
largest_connected_component = separate_largest_connected_component(connected_components)

# 打印最大连通区域的信息
print("largest_connected_component:", len(largest_connected_component))

# 过滤掉特别小的边缘环
filtered_connected_components = [component for component in connected_components
                                 if len(component) >= len(
        largest_connected_component) * 0.1 and component != largest_connected_component]

# 打印过滤后的连通区域数量
print(f"Filtered Connected Components: {len(filtered_connected_components)}")

# =============================================================================
# 导出json文件
data = []

for cluster_index, cluster_data in clusters.items():  # 使用 items() 获取键值对
    cluster_faces = cluster_data['faces']
    cluster_sharp_edges = cluster_data['edge']
    edge_data = []

    # 检查边是否与最大连通区域有交集
    outer_edges_indices = []
    for edge_index in cluster_sharp_edges:
        if edge_index in largest_connected_component:
            outer_edges_indices.append(edge_index)

    outer_edges_components = find_connected_components(bm, set(outer_edges_indices))
    print("outer_edges_components:", len(outer_edges_components))

    # 存储外边缘和内边缘
    outer_edges = [[] for _ in range(len(outer_edges_components))]  # 创建与外边缘连通区域数量相同的空列表
    inner_edges = [[] for _ in range(len(filtered_connected_components))]  # 创建与内边缘环连通区域数量相同的空列表

    for edge_index in cluster_sharp_edges:
        edge = bm.edges[edge_index]
        vert1_index = edge.verts[0].index
        vert2_index = edge.verts[1].index

        # 存储边的顶点索引
        edge_data.append([vert1_index, vert2_index])

        adjacent_faces = [face for face in edge.link_faces if face]

        # 根据连通性来存储外边缘
        if len(outer_edges_indices) != 0:
            for i, component in enumerate(outer_edges_components):
                if edge_index in component:
                    outer_edges[i].append([vert1_index, vert2_index])
                    break
        else:
            # 检查边是否在过滤后的连通区域中
            for i, component in enumerate(filtered_connected_components):
                if edge_index in component:
                    inner_edges[i].append([vert1_index, vert2_index])
                    break
    # 去除inner_edges中为空的元素
    inner_edges = [edge_list for edge_list in inner_edges if edge_list]

    cluster_data = {
        "faces": list(cluster_faces),
        "lines": edge_data,
        "outerLines": outer_edges,  # 存储所有外边缘列表
        "innerLines": inner_edges  # 存储所有内边缘列表
    }

    data.append(cluster_data)

# Export the data to a JSON file
output_file_path = "export/clusters_final.json"
with open(output_file_path, "w") as json_file:
    json.dump(data, json_file, indent=4)

print("Data exported to", output_file_path)
print("over")
bm.free()