INPUT_CAD    = "1372.stp"   # .step/.stp/.iges/.igs
OUT_OBJ      = "slim.obj"
LIN_DEF_MM   = 3.0             # tessellation linear deflection (mm)
ANG_DEF_RAD  = 0.9             # tessellation angular deflection (radians)
TARGET_FACES = 10_000          # desired total faces after decimation
CULL_TINY_MM = 10             # drop parts with bbox diagonal < N mm; or None to disable

import math
import os
import shutil
import tempfile
from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np

try:
    import trimesh
except ImportError as exc:
    raise RuntimeError(
        "trimesh is required for this pipeline. Install it via pip or conda."
    ) from exc

try:
    import pymeshlab  # type: ignore
except ImportError:
    pymeshlab = None  # type: ignore

try:
    import open3d as o3d  # type: ignore
except ImportError:
    o3d = None  # type: ignore

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.IGESControl import IGESControl_Reader
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.gp import gp_Pnt


def read_shape(cad_path: str):
    """
    Read a CAD file via the appropriate OpenCascade reader and return a TopoDS_Shape.
    """
    path = Path(cad_path)
    if not path.exists():
        raise RuntimeError(f"CAD file not found: {path}")

    ext = path.suffix.lower()
    if ext in {".step", ".stp"}:
        reader = STEPControl_Reader()
    elif ext in {".iges", ".igs"}:
        reader = IGESControl_Reader()
    else:
        raise RuntimeError(f"Unsupported CAD extension: {ext}")

    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Failed to read CAD file '{path}'. Status code: {status}.")

    transfer_count = reader.TransferRoots()
    if transfer_count == 0:
        raise RuntimeError(f"No transferable shapes found in '{path}'.")

    shape = reader.Shape()
    if shape.IsNull():
        raise RuntimeError(f"Transferred shape from '{path}' is null.")

    return shape


def tessellate(shape, lin_def_mm: float = 3.0, ang_def_rad: float = 0.9) -> None:
    """
    Tessellate the shape using OpenCascade's incremental mesher.
    """
    if lin_def_mm <= 0:
        raise RuntimeError("lin_def_mm must be positive.")
    if ang_def_rad <= 0 or ang_def_rad > math.pi:
        raise RuntimeError("ang_def_rad must be in (0, pi].")

    mesher = BRepMesh_IncrementalMesh(shape, float(lin_def_mm), False, float(ang_def_rad), True)
    mesher.Perform()
    if hasattr(mesher, "IsDone") and not mesher.IsDone():
        raise RuntimeError("BRepMesh_IncrementalMesh failed to tessellate the shape.")


def extract_triangles(shape) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int, int]]]:
    """
    Extract triangle vertices and face indices from a tessellated shape.
    """
    vertices: List[Tuple[float, float, float]] = []
    faces: List[Tuple[int, int, int]] = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)

    while explorer.More():
        face = explorer.Current()
        loc = TopLoc_Location()
        tri_handle = BRep_Tool.Triangulation(face, loc)
        if tri_handle is None:
            explorer.Next()
            continue
        if hasattr(tri_handle, "IsNull") and tri_handle.IsNull():
            explorer.Next()
            continue

        triangulation = tri_handle.GetObject() if hasattr(tri_handle, "GetObject") else tri_handle
        if triangulation is None:
            explorer.Next()
            continue

        if hasattr(triangulation, "Nodes"):
            nodes = triangulation.Nodes()
            node_lower = nodes.Lower()
            node_upper = nodes.Upper()

            def fetch_node(idx: int):
                return nodes.Value(idx)

        else:
            node_lower = 1
            node_upper = triangulation.NbNodes()

            def fetch_node(idx: int):
                return triangulation.Node(idx)

        if hasattr(triangulation, "Triangles"):
            triangles = triangulation.Triangles()
            tri_lower = triangles.Lower()
            tri_upper = triangles.Upper()

            def fetch_triangle(idx: int):
                return triangles.Value(idx)

        else:
            tri_lower = 1
            tri_upper = triangulation.NbTriangles()

            def fetch_triangle(idx: int):
                return triangulation.Triangle(idx)

        transform = loc.Transformation()
        has_identity = hasattr(transform, "IsIdentity")

        offset = len(vertices)
        for idx in range(node_lower, node_upper + 1):
            point = fetch_node(idx)
            if has_identity:
                if not transform.IsIdentity():
                    point = point.Transformed(transform)
            else:
                point = point.Transformed(transform)
            vertices.append((point.X(), point.Y(), point.Z()))

        face_orientation = face.Orientation()
        for idx in range(tri_lower, tri_upper + 1):
            tri = fetch_triangle(idx)
            n1, n2, n3 = tri.Get()
            if face_orientation == TopAbs_REVERSED:
                n2, n3 = n3, n2
            faces.append((offset + n1 - 1, offset + n2 - 1, offset + n3 - 1))

        explorer.Next()

    if not faces:
        raise RuntimeError("No triangles were extracted; ensure tessellation parameters are adequate.")

    return vertices, faces


def write_obj(path: str, vertices: Sequence[Tuple[float, float, float]], faces: Sequence[Tuple[int, int, int]], scale: float = 0.001) -> None:
    """
    Write a minimal OBJ (positions + triangle faces) scaling milimeters to meters.
    """
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", encoding="utf-8") as obj_file:
        for x, y, z in vertices:
            obj_file.write(f"v {x * scale:.9f} {y * scale:.9f} {z * scale:.9f}\n")
        for a, b, c in faces:
            obj_file.write(f"f {a + 1} {b + 1} {c + 1}\n")


def decimate(inp_path: str, out_path: str, target_faces: int = 80_000) -> str:
    """
    Decimate the mesh using the best available tool, or copy if no decimator is available.
    """
    source = Path(inp_path)
    destination = Path(out_path)
    destination.parent.mkdir(parents=True, exist_ok=True)

    if target_faces is None or target_faces <= 0:
        shutil.copy2(source, destination)
        return "copy (target disabled)"

    try:
        mesh = trimesh.load(source, force="mesh")
        current_faces = int(mesh.faces.shape[0])
    except Exception:
        shutil.copy2(source, destination)
        return "copy (failed to load for decimation)"

    if current_faces <= target_faces:
        shutil.copy2(source, destination)
        return "copy (below target)"

    if pymeshlab is not None:
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(str(source))
        if hasattr(ms, "simplification_quadric_edge_collapse_decimation"):
            ms.simplification_quadric_edge_collapse_decimation(
                targetfacenum=int(target_faces),
                preservenormal=True,
                preserveboundary=True,
                qualitythr=0.3,
            )
            ms.save_current_mesh(str(destination), binary=False)
            return "pymeshlab"
        if hasattr(ms, "apply_filter"):
            try:
                ms.apply_filter(
                    "simplification_quadric_edge_collapse",
                    targetfacenum=int(target_faces),
                    preservenormal=True,
                    preserveboundary=True,
                    qualitythr=0.3,
                )
                ms.save_current_mesh(str(destination), binary=False)
                return "pymeshlab (legacy filter)"
            except Exception:
                pass

    if o3d is not None:
        mesh_o3d = o3d.io.read_triangle_mesh(str(source))
        if mesh_o3d.is_empty():
            shutil.copy2(source, destination)
            return "copy (open3d empty)"
        mesh_o3d.compute_vertex_normals()
        simplified = mesh_o3d.simplify_quadric_decimation(int(target_faces))
        if simplified.is_empty():
            simplified = mesh_o3d
        o3d.io.write_triangle_mesh(str(destination), simplified, write_ascii=True)
        return "open3d"

    shutil.copy2(source, destination)
    return "copy (no decimator)"


def cull_tiny_components(obj_in: str, obj_out: str, min_diag_mm: float = 5.0) -> None:
    """
    Remove connected components with a bounding-box diagonal below the threshold (in millimetres).
    """
    source = Path(obj_in)
    destination = Path(obj_out)
    destination.parent.mkdir(parents=True, exist_ok=True)

    try:
        mesh = trimesh.load(source, force="mesh")
    except Exception as exc:
        raise RuntimeError(f"Failed to load OBJ for culling: {source}") from exc

    if mesh.is_empty or min_diag_mm is None:
        shutil.copy2(source, destination)
        return

    working = mesh.copy()
    working.apply_scale(1000.0)  # meters -> millimeters
    try:
        components = working.split(only_watertight=False)
    except ImportError:
        shutil.copy2(source, destination)
        return

    if not components:
        shutil.copy2(source, destination)
        return

    kept = []
    threshold = float(min_diag_mm)
    for component in components:
        bounds = component.bounds
        if bounds is None or bounds.shape != (2, 3):
            continue
        diag = np.linalg.norm(bounds[1] - bounds[0])
        if diag >= threshold:
            component.apply_scale(0.001)  # back to meters
            kept.append(component)

    if kept:
        combined = kept[0] if len(kept) == 1 else trimesh.util.concatenate(kept)
        combined.export(destination)
    else:
        shutil.copy2(source, destination)

def convert_cad_to_obj(cad_path: str, out_obj: str, lin_def_mm: float, ang_def_rad: float, target_faces: int, cull_tiny_mm):
    """
    Full pipeline: CAD -> tessellated OBJ -> optional culling -> optional decimation.
    """
    cad_path = str(cad_path)
    out_obj = str(out_obj)
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir_path = Path(temp_dir)

        shape = read_shape(cad_path)
        tessellate(shape, lin_def_mm=lin_def_mm, ang_def_rad=ang_def_rad)
        vertices, faces = extract_triangles(shape)

        raw_obj = temp_dir_path / "tessellated.obj"
        write_obj(raw_obj, vertices, faces, scale=0.001)

        pre_decimation_obj = raw_obj
        faces_after_cull = len(faces)

        if cull_tiny_mm is not None:
            culled_obj = temp_dir_path / "culled.obj"
            cull_tiny_components(str(raw_obj), str(culled_obj), float(cull_tiny_mm))
            pre_decimation_obj = culled_obj
            try:
                mesh_after_cull = trimesh.load(pre_decimation_obj, force="mesh")
                faces_after_cull = int(mesh_after_cull.faces.shape[0])
            except Exception:
                faces_after_cull = len(faces)

        decimated_obj = temp_dir_path / "decimated.obj"
        target_faces_value = None if target_faces is None else int(target_faces)
        decimator_used = decimate(str(pre_decimation_obj), str(decimated_obj), target_faces=target_faces_value if target_faces_value is not None else None)

        final_obj_for_export = decimated_obj if decimated_obj.exists() else pre_decimation_obj

        out_obj_path = Path(out_obj)
        out_obj_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(final_obj_for_export, out_obj_path)

        final_faces = faces_after_cull
        try:
            mesh_final = trimesh.load(out_obj_path, force="mesh")
            final_faces = int(mesh_final.faces.shape[0])
        except Exception:
            pass

        obj_size_mb = os.path.getsize(out_obj_path) / (1024 * 1024)

        print(f"Source CAD     : {cad_path}")
        print(f"Initial faces  : {len(faces):,}")
        if cull_tiny_mm is not None:
            print(f"After culling  : {faces_after_cull:,} (threshold {cull_tiny_mm} mm)")
        print(f"Final faces    : {final_faces:,} (decimator: {decimator_used})")
        print(f"OBJ written to : {out_obj_path} ({obj_size_mb:.2f} MB)")


convert_cad_to_obj(
    cad_path=INPUT_CAD,
    out_obj=OUT_OBJ,
    lin_def_mm=LIN_DEF_MM,
    ang_def_rad=ANG_DEF_RAD,
    target_faces=TARGET_FACES,
    cull_tiny_mm=CULL_TINY_MM,
)
