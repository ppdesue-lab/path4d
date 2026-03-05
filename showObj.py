import argparse
import open3d as o3d
import numpy as np

def parse_obj(filename):
    """解析OBJ文件，返回顶点、法线和线条信息"""
    vertices = []
    normals = []
    lines = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if not parts:
                continue
            
            if parts[0] == 'v':
                # 顶点
                x, y, z = map(float, parts[1:4])
                vertices.append([x, y, z])
            elif parts[0] == 'vn':
                # 法线
                nx, ny, nz = map(float, parts[1:4])
                normals.append([nx, ny, nz])
            elif parts[0] == 'l':
                # 线条
                line_vertices = []
                for part in parts[1:]:
                    # 处理顶点索引，格式可能是 '1' 或 '1//1'
                    if '//' in part:
                        vertex_idx = int(part.split('//')[0]) - 1  # OBJ索引从1开始
                    else:
                        vertex_idx = int(part) - 1
                    line_vertices.append(vertex_idx)
                lines.append(line_vertices)
    
    return np.array(vertices), np.array(normals), lines

def plot_obj(vertices, normals, lines):
    """使用Open3D绘制OBJ文件中的线条和法向量"""
    # 创建线条集合
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)

    # 构建线条索引
    line_indices = []
    for line in lines:
        for i in range(len(line) - 1):
            line_indices.append([line[i], line[i+1]])
        #end-1 -> 0
        line_indices.append([line[len(line) - 1], line[0]])
    
    line_set.lines = o3d.utility.Vector2iVector(line_indices)
    # 设置线条颜色为白色
    line_set.colors = o3d.utility.Vector3dVector([[1.0, 1.0, 1.0] for _ in range(len(line_indices))])
    
    # 创建法向量可视化
    normals_vis = []
    normal_length = 0.1
    for i, vertex in enumerate(vertices):
        if i < len(normals):
            # 起点
            start = vertex
            # 终点
            end = vertex + normals[i] * normal_length
            # 创建线段
            line = o3d.geometry.LineSet()
            line.points = o3d.utility.Vector3dVector([start, end])
            line.lines = o3d.utility.Vector2iVector([[0, 1]])
            # 设置颜色为红色
            line.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0]])
            normals_vis.append(line)
    
    # 可视化
    geometries = normals_vis#[line_set]
    geometries.append(line_set)
    # 设置背景为黑色
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="OBJ Lines and Normals")
    for geometry in geometries:
        vis.add_geometry(geometry)
    # 获取渲染选项并设置背景颜色为黑色
    render_option = vis.get_render_option()
    render_option.background_color = [0.0, 0.0, 0.0]  # 黑色背景
    # 运行可视化
    vis.run()
    vis.destroy_window()

def main():
    parser = argparse.ArgumentParser(description='显示OBJ文件中的线条和法向量')
    parser.add_argument('obj_file', help='OBJ文件路径')
    args = parser.parse_args()
    
    vertices, normals, lines = parse_obj(args.obj_file)

    plot_obj(vertices, normals, lines)

if __name__ == '__main__':
    main()
