
from .pyply import _open_stream
import numpy as np
from typing import Tuple
from .pyply import PlyData, PlyElement
from .._core import get_stl_data


def write_ply_xyzrgbnxnynz(ply_filename, points, layout_bgr=False):
    PLY_DTYPE = [
        ("x", "f4"), ("y", "f4"), ("z", "f4"),
        ("red", "u1"), ("green", "u1"), ("blue", "u1"),
        ("nx", "f4"), ("ny", "f4"), ("nz", "f4"),
    ]

    if layout_bgr:
        PLY_DTYPE = [
            ("x", "f4"), ("y", "f4"), ("z", "f4"),
            ("blue", "u1"), ("green", "u1"), ("red", "u1"),
            ("nx", "f4"), ("ny", "f4"), ("nz", "f4"),
        ]
    vertex = np.array([tuple(j) for j in points[:, 0:9]], dtype=PLY_DTYPE)
    PlyData(
        [
            PlyElement.describe(vertex, "vertex", comments=["xyzrgbnxnynz"]),
        ],
        text=False,
        byte_order="=",
        comments=["colored pointcloud with normal"],
    ).write(ply_filename)


def write_ply_xyzrgb(ply_filename, points, layout_bgr=False):
    PLY_DTYPE = [
        ("x", "f4"), ("y", "f4"), ("z", "f4"),
        ("red", "u1"), ("green", "u1"), ("blue", "u1")
    ]

    if layout_bgr:
        PLY_DTYPE = [
            ("x", "f4"), ("y", "f4"), ("z", "f4"),
            ("blue", "u1"), ("green", "u1"), ("red", "u1")

        ]
    vertex = np.array([tuple(j) for j in points[:, 0:6]], dtype=PLY_DTYPE)
    PlyData(
        [
            PlyElement.describe(vertex, "vertex", comments=["xyzrgb"]),
        ],
        text=False,
        byte_order="=",
        comments=["colored pointcloud"],
    ).write(ply_filename)


def write_ply_VF(ply_filename, vertices, faces):
    PLY_DTYPE = [("x", "f4"), ("y", "f4"), ("z", "f4")]
    vertex = np.array([tuple(j) for j in vertices[:, 0:3]], dtype=PLY_DTYPE)
    # PLY_DTYPE = [("x", "f4"), ("y", "f4"), ("z", "f4")]
    ply_faces = np.empty(len(faces),                         dtype=[
                         ('vertex_indices', 'i4', (3,))])
    ply_faces['vertex_indices'] = faces
    vertex = np.array([tuple(j) for j in vertices[:, 0:3]], dtype=PLY_DTYPE)
    PlyData(
        [
            PlyElement.describe(vertex, "vertex", comments=["xyzrgb"]),
            PlyElement.describe(ply_faces, 'face')
        ],
        text=False,
        byte_order="=",
        comments=["colored pointcloud"],
    ).write(ply_filename)


def read_ply_VF(file_path: str) -> tuple([np.ndarray, np.ndarray]):
    """AI is creating summary for read_ply_VF

    Args:
        file_path (str): [description]

    Returns:
        [vertices, faces]: [description]
    """
    vtype = np.float32
    ftype = np.uint32
    ply = PlyData.read(file_path)
    # print(ply.elements)
    list_elements = []
    for elem in ply.elements:
        list_elements.append(elem.name)
    vertices = None
    faces = None

    for elem in ply.elements:
        if (elem.name == "vertex"):
            vertices = np.array(
                elem.data[['x', 'y', 'z']].tolist()).astype(vtype)
        elif (elem.name == "face"):
            ids = []
            for j, _ in enumerate(elem.properties):
                if (j is not None):
                    ids.append(j)
            assert (len(ids) == 1)
            faces = np.array([list(j[0]) for j in elem.data], dtype=ftype)

    return vertices, faces


def read_ply_VN(file_path: str) -> tuple([np.ndarray, np.ndarray]):
    vtype = np.float32
    ply = PlyData.read(file_path)
    list_elements = []
    for elem in ply.elements:
        list_elements.append(elem.name)

    vertices = None
    normals = None
    colors = None
    for elem in ply.elements:
        if (elem.name == "vertex"):
            if ('x' in elem.data.dtype.names):
                vertices = np.array(
                    elem.data[['x', 'y', 'z']].tolist()).astype(vtype)
            if ('nx' in elem.data.dtype.names):
                normals = np.array(elem.data[['nx', 'ny', 'nz']].tolist()).astype(
                    vtype)
            if ('red' in elem.data.dtype.names):
                colors = np.array(elem.data[['blue', 'green', 'red']].tolist()).astype(
                    np.uint8)
            return vertices, colors, normals


def write_ply_xyznxnynz(ply_filename, points):
    PLY_DTYPE = [
        ("x", "f4"), ("y", "f4"), ("z", "f4"),
        ("nx", "f4"), ("ny", "f4"), ("nz", "f4"),
    ]
    vertex = np.array([tuple(j) for j in points[:, 0:6]], dtype=PLY_DTYPE)
    PlyData(
        [
            PlyElement.describe(vertex, "vertex", comments=["xyznxnynz"]),
        ],
        text=False,
        byte_order="=",
        comments=["pointcloud with normal"],
    ).write(ply_filename)


def write_ply_xyz(ply_filename, points):
    PLY_DTYPE = [
        ("x", "f4"), ("y", "f4"), ("z", "f4")
    ]
    vertex = np.array([tuple(j) for j in points[:, 0:3]], dtype=PLY_DTYPE)
    PlyData(
        [
            PlyElement.describe(vertex, "vertex", comments=["xyz"]),
        ],
        text=False,
        byte_order="=",
        comments=["pointcloud"],
    ).write(ply_filename)


def write_trimesh(ply_filename: str, V: np.ndarray, F: np.ndarray):
    """write a V,F triangle mesh to a binary .ply
    Args:
        ply_filename (str): [description]
        V (np.ndarray): (nV,3) float32
        F (np.ndarray): (nF,3) uint32
    """
    write_ply_VF(ply_filename, V, F)


def read_trimesh(trimesh_filename: str):
    """read a V,F triangle mesh from a binary .ply

    Args:
        file_path (str): [description]

    Returns:
        (vertices,faces) (np.ndarray, np.ndarray): vertices = (nV,3) float32 and faces = (nF,3) uint32
    """

    import os
    _, ext = os.path.splitext(trimesh_filename)

    if ext.casefold() == ".ply".casefold():
        return read_ply_VF(trimesh_filename)
    elif ext.casefold() == ".stl".casefold():
        return get_stl_data(trimesh_filename)
    else:
        return None, None


def read_pointcloud(ply_filename: str):
    """read a V,N,C pointcloud from a binary .ply

    Args:
        ply_filename (str): [description]

    Returns:
        (vertices,colors, normals) (np.ndarray, np.ndarray, np.ndarray): vertices = (nV,3) float32 , colors = (nV,3) uint8 and normals = (nV,3) float32
    """

    return read_ply_VN(ply_filename)


def write_pointcloud(ply_filename: str, xyzI: np.ndarray, colorI: np.ndarray = None, normalI: np.ndarray = None, is_bgr: bool = False):
    """write a rgbd image to a binary .ply
    Args:
        ply_filename (str): _description_
        xyzI (np.ndarray): (h,w,3) float32
        colorI (np.ndarray): (h,w,3) uint8
        normalI (np.ndarray): (h,w,3) float32
        is_bgr (bool, optional): _description_. Defaults to False.
    """

    if colorI is None:
        has_color = False
    else:
        has_color = colorI.any()
    if normalI is None:
        has_normal = False
    else:
        has_normal = normalI.any()

    if (has_color and has_normal):
        xyzRGBnxnynz = np.hstack(
            [xyzI.reshape((-1, 3)), colorI.reshape((-1, 3)), normalI.reshape((-1, 3))])
        write_ply_xyzrgbnxnynz(ply_filename, xyzRGBnxnynz, is_bgr)
        return
    if (has_color and not has_normal):
        xyzRGB = np.hstack([xyzI.reshape((-1, 3)), colorI.reshape((-1, 3))])
        write_ply_xyzrgb(ply_filename, xyzRGB, is_bgr)
        return
    if (not has_color and has_normal):
        xyznxnynz = np.hstack(
            [xyzI.reshape((-1, 3)), normalI.reshape((-1, 3))])
        write_ply_xyznxnynz(ply_filename, xyznxnynz)
        return
    if (not has_color and not has_normal):
        write_ply_xyz(ply_filename, xyzI.reshape((-1, 3)))
        return


def write_polylines(stream, verts, lines, vertex_normal=None):
    """
    Write polylines to an obj file.
    Note: Each line in the file only contains one segment so that Meshlab
    opens it properly.

    Parameters
    ----------
    fn : str
        path to the output obj file
    verts : numpy.array
        vertex coordinates (n x 3)
    lines : list of numpy.array
        list of curves [arr_0, arr_1, ...]
        The i-th curve, arr_i, is a vector of vertex indices of vertices on it,
        0-based.
    Returns
    -------
    None
    """

    # with open(fn, 'w') as f:
    (must_close, stream) = _open_stream(stream, "write")

    for v in verts:
        stream.write(('v %.8g %.8g %.8g\n' %
                     (v[0], v[1], v[2])).encode("ascii"))
    # print(len(vertex_normal))
    if vertex_normal is not None:
        if len(vertex_normal) == len(verts):
            for vn in vertex_normal:
                stream.write(('vn %.8g %.8g %.8g\n' %
                             (vn[0], vn[1], vn[2])).encode("ascii"))

    for line in lines:
        for i in range(len(line)-1):
            stream.write(('l %d//%d %d//%d\n' %
                         (line[i]+1, line[i]+1, line[i+1]+1, line[i+1]+1)).encode("ascii"))
    # for line in lines:
    #     for i in range(len(line)-1):
    #         stream.write(('f %d//%d %d//%d %d//%d\n' % (line[i]+1,line[i]+1, line[i+1]+1, line[i+1]+1 , 1,1)).encode("ascii"))

    if must_close:
        stream.close()


def read_polylines(fn):
    """
    Read polylines from an obj file.

    Parameters
    ----------
    fn : str
        path to the output obj file
    Returns
    -------
    verts : numpy.array
        vertex coordinates (n x 3)
    lines : numpy.array (m x 2)
        edge segments
    """
    verts = []
    segs = []
    with open(fn, 'r') as f:
        lines = f.readlines()
    for i in range(len(lines)):
        s = lines[i].split()
        if (len(s) == 0):
            continue
        if (s[0] == 'v'):
            verts.append([float(s[1]), float(s[2]), float(s[3])])
        elif (s[0] == 'l'):
            L = list(map(lambda x: int(x),  s[1:]))
            segs.extend([[L[i], L[i+1]] for i in range(len(L)-1)])

    verts_mat = np.array(verts)
    segs_mat = np.array(segs)-1  # 1-indexed -> 0-indexed
    return verts_mat, segs_mat
