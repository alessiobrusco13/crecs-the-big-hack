# CAD → OBJ Pipeline

Convert STEP/IGES CAD files into lightweight **OBJ** assets ready for Unity/VR.  
This pipeline uses **pythonOCC** for tessellation, optional **pymeshlab/open3d** for decimation, and **trimesh** for mesh utilities.

---

## Features
- ✅ STEP/IGES input → OBJ output
- ✅ mm → meters scaling for Unity
- ✅ Optional tiny-part culling (by bbox diagonal, in mm)
- ✅ Optional decimation to a target triangle budget
- ✅ Minimal dependencies; robust fallbacks

---

## Requirements
Install in your conda/mamba env (prefer conda-forge):

```bash
mamba install -c conda-forge pythonocc-core trimesh pymeshlab open3d
# pymeshlab and/or open3d are optional (for decimation)
```

---

## Quick Start
1. Put your CAD file (e.g., `1372.stp`) next to the script/notebook.
2. Set the config values (see below).
3. Run the notebook cell (or function call) to generate `slim.obj`.

---

## Config Parameters

```python
INPUT_CAD    = "1372.stp"   # .step/.stp/.iges/.igs
OUT_OBJ      = "slim.obj"
LIN_DEF_MM   = 3.0          # tessellation linear deflection (mm)
ANG_DEF_RAD  = 0.9          # tessellation angular deflection (radians)
TARGET_FACES = 80_000       # desired total faces after decimation
CULL_TINY_MM = 5.0          # drop parts with bbox diagonal < N mm; or None to disable
```

### What they do
- **INPUT_CAD**: Path to your CAD file (STEP/IGES).  
- **OUT_OBJ**: Output OBJ filename.  
- **LIN_DEF_MM** *(mm)*: Tessellation coarseness. **Higher** ⇒ fewer triangles (faster/lighter).  
  - 2.0 (detailed), **3.0 (default)**, 4–5 (lighter for VR)  
- **ANG_DEF_RAD** *(radians)*: Angular coarseness. **Higher** ⇒ fewer triangles. Typical **0.6–0.9**; **0.9** is coarse & fast.  
- **TARGET_FACES**: Post-tessellation triangle budget for decimation. If already below, mesh is copied unchanged.  
  - Standalone VR comfort: **50k–100k** visible triangles.  
- **CULL_TINY_MM** *(mm)*: Drop connected components with bbox diagonal below this (screws/threads). Set `None` to disable.

---

## Pipeline
1. **Read CAD** → `TopoDS_Shape` (STEP/IGES).  
2. **Tessellate** with `BRepMesh_IncrementalMesh` using `LIN_DEF_MM`, `ANG_DEF_RAD`.  
3. **Extract triangles** via `BRep_Tool.Triangulation(face, TopLoc_Location())` (apply transforms & face orientation).  
4. **Write OBJ** (positions + faces only), scaled **mm → m**.  
5. **Cull tiny parts** *(optional)* using `CULL_TINY_MM`.  
6. **Decimate** *(optional)* to `TARGET_FACES` using **pymeshlab** (preferred) or **open3d**, else copy.  
7. **Copy final OBJ** to `OUT_OBJ`.

---

## Core Functions (overview)
- `read_shape(path)`: Load STEP/IGES and return `TopoDS_Shape`.  
- `tessellate(shape, lin_def_mm, ang_def_rad)`: OCC meshing; larger values ⇒ coarser mesh.  
- `extract_triangles(shape) -> (vertices, faces)`: Iterate faces, apply transform/orientation, gather triangles.  
- `write_obj(path, vertices, faces, scale=0.001)`: Minimal OBJ writer, scales mm→m.  
- `cull_tiny_components(obj_in, obj_out, min_diag_mm)`: Remove small parts by bbox diagonal.  
- `decimate(inp_path, out_path, target_faces)`: pymeshlab → open3d → copy.  
- `convert_cad_to_obj(...)`: Orchestrates the whole pipeline and prints a summary.

---

## Tuning Recipes
**Too heavy / low FPS**
- Increase `LIN_DEF_MM` → **4–5**
- Lower `TARGET_FACES` → **60,000** (or 40–50k)
- Increase `CULL_TINY_MM` → **10–20**

**Too blocky / detail loss**
- Decrease `LIN_DEF_MM` → **2.0–2.5**
- Increase `TARGET_FACES` → **100–150k** (device permitting)

**Tiny fasteners dominate triangles**
- Raise `CULL_TINY_MM` → **15 mm** or more

---

## Notes
- Output OBJ is already in **meters**, so Unity scale is correct.  
- Face orientation is handled; triangle winding stays consistent.  
- `trimesh` is required; `pymeshlab`/`open3d` are optional for decimation.  
- If you see SciPy warnings in `trimesh.process`, export still works.

---

## Example (function call)
```python
convert_cad_to_obj(
    cad_path=INPUT_CAD,
    out_obj=OUT_OBJ,
    lin_def_mm=LIN_DEF_MM,
    ang_def_rad=ANG_DEF_RAD,
    target_faces=TARGET_FACES,
    cull_tiny_mm=CULL_TINY_MM,
)
```

---

## License
MIT.
