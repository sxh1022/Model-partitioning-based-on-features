# 根据参数选择特征

import bpy
import bmesh
import math
import mathutils


# 计算两个面之间的二面角（法线之间的夹角）
def calc_dihedral_angle(face1, face2):
    normal1 = face1.normal
    normal2 = face2.normal
    dot_product = normal1.dot(normal2)
    angle_rad = math.acos(min(max(dot_product, -1.0), 1.0))
    return math.degrees(angle_rad)


# 计算连接边的两个顶点之间的距离以及连接到这两个顶点的所有边中的最大直径
def calc_shape_diameter(edge):
    vertex1 = edge.verts[0]
    vertex2 = edge.verts[1]
    distance = (vertex1.co - vertex2.co).length

    connected_edges = set(vertex1.link_edges) | set(vertex2.link_edges)
    max_diameter = distance
    for connected_edge in connected_edges:
        if connected_edge != edge:
            connected_vertex1 = connected_edge.verts[0]
            connected_vertex2 = connected_edge.verts[1]
            connected_distance = (connected_vertex1.co - connected_vertex2.co).length
            max_diameter = max(max_diameter, connected_distance)

    return max_diameter


# 进行预处理，计算并返回各种阈值
def preprocess_model(bm):
    total_normal_angle_deg = 0
    total_shape_diameter = 0
    total_curvature = 0
    total_dihedral_angle = 0
    num_edges_with_2_faces = 0

    for face in bm.faces:
        normal_angle_deg = face.normal.angle(mathutils.Vector((0, 0, 1)))
        total_normal_angle_deg += math.degrees(normal_angle_deg)

    for edge in bm.edges:
        total_shape_diameter += calc_shape_diameter(edge)

        if len(edge.link_faces) == 2:
            total_curvature += abs(edge.calc_face_angle_signed())
            num_edges_with_2_faces += 1
            face1, face2 = edge.link_faces
            total_dihedral_angle += calc_dihedral_angle(face1, face2)

    num_faces = len(bm.faces)
    num_edges = len(bm.edges)
    average_normal_angle_deg = total_normal_angle_deg / num_faces
    average_shape_diameter = total_shape_diameter / num_edges
    average_curvature = total_curvature / num_edges
    average_dihedral_angle = total_dihedral_angle / num_edges_with_2_faces

    angle_threshold_deg = min(20, average_normal_angle_deg)
    shape_diameter_threshold = 0.5 * average_shape_diameter
    curvature_threshold = 0.5 * average_curvature
    dihedral_threshold = 0.5 * average_dihedral_angle

    return curvature_threshold, angle_threshold_deg, dihedral_threshold, shape_diameter_threshold


# 获取活动对象
obj = bpy.context.active_object
# 获取网格数据
mesh = obj.data
mesh.materials.clear()

# 创建BMesh实例
bm = bmesh.new()
bm.from_mesh(mesh)

# 切换到编辑模式，设置选择模式为边
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_mode(type="EDGE")
bpy.ops.mesh.select_all(action='DESELECT')
bpy.ops.object.mode_set(mode='OBJECT')

# 进行预处理，得到参数阈值
curvature_threshold, angle_threshold_deg, dihedral_threshold, shape_diameter_threshold = preprocess_model(bm)

# 输出参数信息
print("Curvature Threshold:", curvature_threshold)
print("Angle Threshold (degrees):", angle_threshold_deg)
print("Dihedral Threshold:", dihedral_threshold)
print("Shape Diameter Threshold:", shape_diameter_threshold)

# 标记锐边
for edge in bm.edges:
    edge.select = False
    if edge.is_boundary:
        edge.select = True
    elif len(edge.link_faces) == 2:
        face1, face2 = edge.link_faces
        normal_angle_rad = face1.normal.angle(face2.normal)
        normal_angle_deg = math.degrees(normal_angle_rad)
        curvature = edge.calc_face_angle_signed()

        is_sharp = (curvature > curvature_threshold and normal_angle_deg > angle_threshold_deg) or \
                   (calc_dihedral_angle(face1, face2) > dihedral_threshold and calc_shape_diameter(
                       edge) < shape_diameter_threshold)

        if is_sharp:
            edge.select = True

# 输出锐边数量
num_sharp_edges = len([edge for edge in bm.edges if edge.select])
print("Number of sharp edges:", num_sharp_edges)

# 更新网格，切换到编辑模式，释放BMesh实例
bm.to_mesh(mesh)
bpy.ops.object.mode_set(mode='EDIT')
bm.free()

print("over")