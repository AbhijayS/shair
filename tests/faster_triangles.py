from os import read
import numpy as np
import trimesh
import struct

def read_stl(filename=None):
    '''
    returns:
        - tuple of points, normals, and triangles
        - points, normals, and triangles are numpy arrays
        - points.shape => (num_points, 3)
        - normals.shape => (num_normals, 3)
        - triangles.shape => (num_triangles, 3)
            - each triangle contains 3 point indices (referencing the points array)
    '''
    
    mesh = trimesh.load_mesh('tests/sphere.stl')
    adjacent_faces = {}

    for a in mesh.face_adjacency:
        if not a[0] in adjacent_faces:
            adjacent_faces[a[0]] = a[1]
        else:
            adjacent_faces[a[0]] = np.append(adjacent_faces[a[0]], a[1])
        if not a[1] in adjacent_faces:
            adjacent_faces[a[1]] = a[0]
        else:
            adjacent_faces[a[1]] = np.append(adjacent_faces[a[1]], a[0])

    # print(adjacent_faces)
    # print(mesh.vertices)
    # print(mesh.faces)

    # print(mesh.triangles[0])

    points = mesh.vertices
    normals = trimesh.triangles.normals(mesh.triangles)[0]
    triangles = mesh.faces
    return (points, normals, triangles, adjacent_faces)
    
    with open(filename, 'rb') as f:
        f.seek(80)
        [facets, ] = struct.unpack('I', f.read(4))
        points = []
        triangles = []
        normals = []

        for i in range(facets):
            normal = list(struct.unpack('fff', f.read(12)))
            normals.append(normal)

            v1 = list(struct.unpack('fff', f.read(12)))
            v2 = list(struct.unpack('fff', f.read(12)))
            v3 = list(struct.unpack('fff', f.read(12)))
            triangle = []

            if v1 in points:
                triangle.append(points.index(v1))
            else:
                triangle.append(len(points))
                points.append(v1)

            if v2 in points:
                triangle.append(points.index(v2))
            else:
                triangle.append(len(points))
                points.append(v2)

            if v3 in points:
                triangle.append(points.index(v3))
            else:
                triangle.append(len(points))
                points.append(v3)

            triangles.append(triangle)
            f.seek(2, 1)
    return (np.array(points), np.array(normals), np.array(triangles))

s = read_stl('tests/sphere.stl')
print(s[2][0])
print(s[2][1])
print(trimesh.graph.shared_edges([s[2][0]], [s[2][1]]))