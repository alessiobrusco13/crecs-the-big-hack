"""Utilities for converting CAD files (STEP/IGES) to OBJ meshes.

The pipeline implemented here performs four stages:
1. Read and tessellate the CAD geometry via OpenCascade.
2. Emit a raw OBJ file (millimetres scaled to metres).
3. Optionally cull tiny connected components to prune noise.
4. Optionally run mesh decimation via PyMeshLab or Open3D.

The entry point is :func:`convert_cad_to_obj` which coordinates the flow,
while the CLI in :func:`main` provides an ergonomic wrapper for batch use.
"""

import argparse
import math
import os
import shutil
import tempfile
import time
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple, TypeVar

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

LIN_DEF_MM: float = 3.0
"""Default tessellation linear deflection (millimetres)."""

ANG_DEF_RAD: float = 0.9
"""Default tessellation angular deflection (radians)."""

TARGET_FACES: int = 30_000
"""Desired total number of faces after decimation."""

CULL_TINY_MM: float = 5.0
"""Maximum bounding-box diagonal (millimetres) for removing components."""

Point3D = Tuple[float, float, float]
Triangle = Tuple[int, int, int]
MeshData = Tuple[List[Point3D], List[Triangle]]
MeshDataLike = Tuple[Sequence[Point3D], Sequence[Triangle]]
NumberT = TypeVar("NumberT", int, float)


def read_shape(cad_path: str):
    """
    Load a CAD file and return an OpenCascade shape ready for tessellation.

    Args:
        cad_path: Path to a STEP (.step/.stp) or IGES (.iges/.igs) file.

    Returns:
        The deserialized ``TopoDS_Shape`` instance provided by OpenCascade.

    Raises:
        RuntimeError: If the file is missing, has an unsupported extension,
            or fails to transfer a valid shape.
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


def tessellate(shape, lin_def_mm: float = LIN_DEF_MM, ang_def_rad: float = ANG_DEF_RAD) -> None:
    """
    Tessellate a CAD shape using OpenCascade's incremental mesher.

    Args:
        shape: OpenCascade ``TopoDS_Shape`` to tessellate in-place.
        lin_def_mm: Linear deflection parameter expressed in millimetres.
        ang_def_rad: Angular deflection parameter expressed in radians.

    Raises:
        RuntimeError: If the tessellation parameters are invalid or
            the mesher reports failure.
    """
    if lin_def_mm <= 0:
        raise RuntimeError("lin_def_mm must be positive.")
    if ang_def_rad <= 0 or ang_def_rad > math.pi:
        raise RuntimeError("ang_def_rad must be in (0, pi].")

    mesher = BRepMesh_IncrementalMesh(shape, float(lin_def_mm), False, float(ang_def_rad), True)
    mesher.Perform()
    if hasattr(mesher, "IsDone") and not mesher.IsDone():
        raise RuntimeError("BRepMesh_IncrementalMesh failed to tessellate the shape.")


def extract_triangles(shape) -> MeshData:
    """
    Extract triangle vertices and face indices from a tessellated shape.

    Args:
        shape: Tessellated OpenCascade ``TopoDS_Shape`` instance.

    Returns:
        Tuple of ``(vertices, faces)`` describing the mesh in millimetres.

    Raises:
        RuntimeError: If no triangles can be extracted from the shape.
    """
    vertices: List[Point3D] = []
    faces: List[Triangle] = []
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


def write_obj(
    path: str,
    vertices: Sequence[Point3D],
    faces: Sequence[Triangle],
    scale: float = 0.001,
) -> None:
    """
    Write a minimal OBJ (positions + triangle faces) and apply unit scaling.

    Args:
        path: Destination OBJ file path.
        vertices: Iterable of vertex positions expressed in millimetres.
        faces: Iterable of triangle indices referencing ``vertices``.
        scale: Factor applied to convert vertices before writing
            (defaults to millimetres → metres).
    """
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", encoding="utf-8") as obj_file:
        for x, y, z in vertices:
            obj_file.write(f"v {x * scale:.9f} {y * scale:.9f} {z * scale:.9f}\n")
        for a, b, c in faces:
            obj_file.write(f"f {a + 1} {b + 1} {c + 1}\n")


def decimate(inp_path: str, out_path: str, target_faces: Optional[int] = 80_000) -> str:
    """
    Reduce the face count of a mesh using the best available backend.

    Args:
        inp_path: Source mesh path that should be simplified.
        out_path: Destination path where the processed mesh is written.
        target_faces: Desired number of faces; ``None`` or ``<= 0`` disables decimation.

    Returns:
        Short description of the backend used or why the mesh was copied verbatim.
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


def cull_tiny_components(obj_in: str, obj_out: str, min_diag_mm: Optional[float] = CULL_TINY_MM) -> None:
    """
    Remove connected components whose bounding-box diagonal falls below ``min_diag_mm``.

    Args:
        obj_in: Path to the source OBJ file (in metres).
        obj_out: Destination path where the filtered OBJ will be written.
        min_diag_mm: Bounding-box diagonal threshold in millimetres.
            When ``None`` the mesh is copied as-is.

    Raises:
        RuntimeError: If the OBJ cannot be loaded for processing.
    """
    source = Path(obj_in)
    destination = Path(obj_out)
    destination.parent.mkdir(parents=True, exist_ok=True)

    if min_diag_mm is None:
        shutil.copy2(source, destination)
        return

    try:
        mesh = trimesh.load(source, force="mesh")
    except Exception as exc:
        raise RuntimeError(f"Failed to load OBJ for culling: {source}") from exc

    if mesh.is_empty:
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


def tessellate_to_mesh(cad_path: str, lin_def_mm: float, ang_def_rad: float) -> MeshData:
    """
    Read and tessellate a CAD file, returning its triangle mesh.

    Args:
        cad_path: Path to the CAD file to process.
        lin_def_mm: Linear deflection in millimetres used for tessellation.
        ang_def_rad: Angular deflection in radians used for tessellation.

    Returns:
        Tuple ``(vertices, faces)`` representing the tessellated mesh in millimetres.
    """
    shape = read_shape(cad_path)
    tessellate(shape, lin_def_mm=lin_def_mm, ang_def_rad=ang_def_rad)
    return extract_triangles(shape)


def convert_cad_to_obj(
    cad_path: str,
    out_obj: str,
    lin_def_mm: float,
    ang_def_rad: float,
    target_faces: Optional[int],
    cull_tiny_mm: Optional[float],
    precomputed_mesh: Optional[MeshDataLike] = None,
) -> None:
    """
    Execute the full CAD → OBJ pipeline including optional post-processing.

    Args:
        cad_path: Path to the source CAD file.
        out_obj: Destination path where the final OBJ should be written.
        lin_def_mm: Linear deflection to use during tessellation (millimetres).
        ang_def_rad: Angular deflection to use during tessellation (radians).
        target_faces: Desired number of faces after decimation (``None`` disables).
        cull_tiny_mm: Bounding-box diagonal threshold in millimetres for component culling.
        precomputed_mesh: Optional mesh data to reuse instead of tessellating again.
    """
    cad_path = str(cad_path)
    out_obj = str(out_obj)
    start_time = time.perf_counter()
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir_path = Path(temp_dir)

        if precomputed_mesh is None:
            vertices, faces = tessellate_to_mesh(cad_path, lin_def_mm=lin_def_mm, ang_def_rad=ang_def_rad)
        else:
            vertices, faces = precomputed_mesh

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
        decimator_used = decimate(str(pre_decimation_obj), str(decimated_obj), target_faces=target_faces)

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
        total_time = time.perf_counter() - start_time
        print(f"Total runtime  : {total_time:.2f} s")


def parse_optional_int_value(value: str) -> Optional[int]:
    """
    Parse an optional integer from CLI input, allowing textual 'none' to disable.
    """
    stripped = value.strip()
    if not stripped:
        return None
    lowered = stripped.lower()
    if lowered in {"none", "null", "off", "disable"}:
        return None
    return int(stripped)


def parse_optional_float_value(value: str) -> Optional[float]:
    """
    Parse an optional float from CLI input, allowing textual 'none' to disable.
    """
    stripped = value.strip()
    if not stripped:
        return None
    lowered = stripped.lower()
    if lowered in {"none", "null", "off", "disable"}:
        return None
    return float(stripped)


def default_output_for(cad_path: str) -> str:
    """
    Derive a default OBJ output path next to this script using the CAD file stem.

    Args:
        cad_path: Path to the CAD file provided by the user.

    Returns:
        Absolute string path for the suggested OBJ output.
    """
    script_dir = Path(__file__).resolve().parent
    cad_name = Path(cad_path).stem or Path(cad_path).name
    return str(script_dir / f"{cad_name}.obj")


def build_arg_parser() -> argparse.ArgumentParser:
    """
    Construct the argument parser used by the CLI entry point.

    Returns:
        Configured ``argparse.ArgumentParser`` instance.
    """
    parser = argparse.ArgumentParser(
        description="Convert STEP/IGES CAD files to OBJ meshes with optional decimation."
    )
    parser.add_argument("--cad", required=True, help="Path to the input CAD file.")
    parser.add_argument(
        "--out",
        default=None,
        help="Path to the output OBJ file (defaults to <script_dir>/<input_stem>.obj).",
    )
    parser.add_argument(
        "--lin-def",
        type=float,
        default=LIN_DEF_MM,
        help="Linear deflection for tessellation in millimetres.",
    )
    parser.add_argument(
        "--ang-def",
        type=float,
        default=ANG_DEF_RAD,
        help="Angular deflection for tessellation in radians.",
    )
    parser.add_argument(
        "--target-faces",
        type=parse_optional_int_value,
        default=TARGET_FACES,
        help="Desired face count after decimation (use 'none' to disable).",
    )
    parser.add_argument(
        "--cull-tiny",
        type=parse_optional_float_value,
        default=CULL_TINY_MM,
        help="Cull components with bounding-box diagonal below this threshold in mm (use 'none' to disable).",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Prompt for parameters after previewing tessellation.",
    )
    return parser


def prompt_text(prompt: str, default: str) -> str:
    """
    Prompt the user for text input, falling back to a default when empty.
    """
    response = input(f"{prompt} [{default}]: ").strip()
    return response or default


def prompt_float(prompt: str, default: float) -> float:
    """
    Prompt the user for a floating-point value, retrying until valid.
    """
    while True:
        response = input(f"{prompt} [{default}]: ").strip()
        if not response:
            return default
        try:
            return float(response)
        except ValueError:
            print("Please enter a valid number.")


def prompt_optional_number(
    prompt: str,
    default: Optional[NumberT],
    caster: Callable[[str], NumberT],
) -> Optional[NumberT]:
    """
    Prompt for an optional numeric value where textual 'none' disables the option.

    Args:
        prompt: Text shown to the user.
        default: Value returned when input is empty.
        caster: Callable converting the non-empty input string to the expected type.

    Returns:
        Parsed value or ``None`` when the user explicitly disables the option.
    """
    display_default = "none" if default is None else str(default)
    while True:
        response = input(f"{prompt} [{display_default}]: ").strip()
        if not response:
            return default
        lowered = response.lower()
        if lowered in {"none", "null", "off", "disable"}:
            return None
        try:
            return caster(response)
        except ValueError:
            print("Please enter a valid value or 'none'.")


def main() -> None:
    """CLI entry point that orchestrates argument parsing and pipeline execution."""
    parser = build_arg_parser()
    args = parser.parse_args()

    cad_path: str = args.cad
    out_provided = args.out is not None
    out_obj = args.out if out_provided else default_output_for(cad_path)
    lin_def_default = float(args.lin_def)
    if lin_def_default <= 0:
        parser.error("--lin-def must be positive.")

    ang_def_default = float(args.ang_def)
    if not (0 < ang_def_default <= math.pi):
        parser.error("--ang-def must be in the range (0, pi].")

    target_faces_default = args.target_faces
    cull_tiny_default = args.cull_tiny
    if cull_tiny_default is not None and cull_tiny_default <= 0:
        parser.error("--cull-tiny must be positive or 'none'.")

    preview_mesh = tessellate_to_mesh(cad_path, lin_def_mm=lin_def_default, ang_def_rad=ang_def_default)
    preview_faces = len(preview_mesh[1])
    print(f"Preview faces  : {preview_faces:,} (lin_def={lin_def_default} mm, ang_def={ang_def_default} rad)")

    if not args.interactive:
        convert_cad_to_obj(
            cad_path=cad_path,
            out_obj=out_obj,
            lin_def_mm=lin_def_default,
            ang_def_rad=ang_def_default,
            target_faces=target_faces_default,
            cull_tiny_mm=cull_tiny_default,
            precomputed_mesh=preview_mesh,
        )
        return

    cad_input = prompt_text("Input CAD file", cad_path)
    cad_path_changed = cad_input != cad_path
    cad_path = cad_input

    if cad_path_changed:
        preview_mesh = tessellate_to_mesh(cad_path, lin_def_mm=lin_def_default, ang_def_rad=ang_def_default)
        preview_faces = len(preview_mesh[1])
        print(f"Preview faces  : {preview_faces:,} (lin_def={lin_def_default} mm, ang_def={ang_def_default} rad)")

    if not out_provided:
        out_obj = default_output_for(cad_path)
    out_obj = prompt_text("Output OBJ file", out_obj)

    while True:
        lin_def = prompt_float("Linear deflection (mm)", lin_def_default)
        if lin_def > 0:
            break
        print("Linear deflection must be positive.")

    while True:
        ang_def = prompt_float("Angular deflection (radians)", ang_def_default)
        if 0 < ang_def <= math.pi:
            break
        print("Angular deflection must be within (0, pi].")

    while True:
        target_faces = prompt_optional_number(
            "Target face count (enter 'none' to disable)",
            target_faces_default,
            int,
        )
        if target_faces is None or target_faces > 0:
            break
        print("Target face count must be positive or 'none'.")

    while True:
        cull_tiny = prompt_optional_number(
            "Cull components below this bounding-box diagonal in mm (enter 'none' to disable)",
            cull_tiny_default,
            float,
        )
        if cull_tiny is None or cull_tiny > 0:
            break
        print("Cull threshold must be positive or 'none'.")

    precomputed_mesh = preview_mesh
    if not math.isclose(lin_def, lin_def_default) or not math.isclose(ang_def, ang_def_default):
        precomputed_mesh = tessellate_to_mesh(cad_path, lin_def_mm=lin_def, ang_def_rad=ang_def)
        print(f"Updated Tessellation: {len(precomputed_mesh[1]):,} faces.")
    else:
        print("Reusing preview tessellation for conversion.")

    convert_cad_to_obj(
        cad_path=cad_path,
        out_obj=out_obj,
        lin_def_mm=lin_def,
        ang_def_rad=ang_def,
        target_faces=target_faces,
        cull_tiny_mm=cull_tiny,
        precomputed_mesh=precomputed_mesh,
    )


if __name__ == "__main__":
    main()
