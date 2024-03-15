# 移除边缘部分的特征

import bpy
import bmesh

boundary_distance_threshold=0.01

# 获取活动对象
obj = bpy.context.active_object
# 获取网格数据
mesh = obj.data

# 创建一个BMesh实例
bm = bmesh.new()
bm.from_mesh(mesh)

# 计算点到最近边界边的距离
def distance_to_boundary(vert, boundary_edges):
    min_distance = float('inf')
    for edge in boundary_edges:
        for v in edge.verts:
            distance = (vert.co - v.co).length
            if distance < min_distance:
                min_distance = distance
    return min_distance

sharp_edges = [edge for edge in bm.edges if edge.select]
boundary_edges = [edge for edge in bm.edges if edge.is_boundary]
print("before remove boundary:",len(sharp_edges))

# 切换到对象模式进行选择
bpy.ops.object.mode_set(mode='OBJECT')

# 取消选择离边缘近的锐边
for edge in sharp_edges:
    if edge.is_boundary:
        continue
    else:
        min_distance = float('inf')
        for vert in edge.verts:
            distance = distance_to_boundary(vert, boundary_edges)
            min_distance = min(min_distance, distance)
        if min_distance < boundary_distance_threshold:
            edge.select = False

# 更新网格
bm.to_mesh(mesh)
# 切换到编辑模式
bpy.ops.object.mode_set(mode='EDIT')
print("after remove boundary:", len([edge for edge in bm.edges if edge.select]))

bm.free()
print("over")