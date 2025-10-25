# Pipeline CAD → OBJ

Converti file CAD STEP/IGES in asset **OBJ** leggeri pronti per motori in tempo reale.  
Il flusso usa **pythonOCC** per la tassellazione, **trimesh** per l'ispezione della mesh e la decimazione opzionale tramite **PyMeshLab** o **Open3D**.

---

## Installazione

Crea (o attiva) un ambiente Python 3.9+ e installa i pacchetti richiesti.  
`pythonocc-core`, `trimesh` e `numpy` sono obbligatori; le funzionalità di decimazione diventano disponibili quando `pymeshlab` e/o `open3d` sono installati.

### Conda / Mamba (consigliato)
```bash
mamba install -c conda-forge pythonocc-core trimesh numpy pymeshlab open3d
```

### pip (assicurati che le librerie di sistema OpenCascade siano disponibili)
```bash
pip install pythonocc-core trimesh numpy
pip install pymeshlab open3d  # opzionali, abilitano i backend di decimazione
```

> Se hai bisogno solo della tassellazione senza semplificazione puoi omettere `pymeshlab` e `open3d`.

---

## Avvio rapido
```bash
python cad_to_obj.py --cad path/to/model.step --out path/to/model.obj
```

- Se ometti `--out` l'OBJ viene salvato accanto allo script usando lo stem del CAD.  
- Aggiungi `--interactive` per visualizzare un'anteprima della tassellazione e regolare i parametri dal terminale.  
- Esegui `python cad_to_obj.py --help` per l'elenco completo delle opzioni.

---

## Modelli di test

La cartella `test_models/` contiene alcuni STEP dimostrativi scaricati da [CADgrab](https://www.cadgrab.com/).  
Sono inclusi come riferimento per validare rapidamente la pipeline; verifica sempre le condizioni d'uso originali prima di distribuirli.

---

## Parametri principali

| Flag | Default | Significato |
|------|---------|-------------|
| `--lin-def` | `3.0` mm | Deviazione lineare per la tassellazione (valori maggiori → meno triangoli). |
| `--ang-def` | `0.9` rad | Deviazione angolare per la tassellazione (valori maggiori → meno triangoli). |
| `--target-faces` | `500000` | Numero di facce desiderato dopo la decimazione (`none` disattiva). |
| `--cull-tiny` | `5.0` mm | Rimuove componenti con diagonale del bounding box inferiore a questo (`none` disattiva). |

Questi valori predefiniti corrispondono alle costanti esposte in `cad_to_obj.py`, così i chiamanti programmatici possono riutilizzare `convert_cad_to_obj(...)` senza configurazione.

---

## Panoramica della pipeline
1. **Leggi il CAD** → `TopoDS_Shape` tramite i reader OpenCascade.  
2. **Tassella** con `BRepMesh_IncrementalMesh`.  
3. **Estrai i triangoli** usando `BRep_Tool.Triangulation(...)`, applicando trasformazioni e orientamento delle facce.  
4. **Scrivi l'OBJ** (posizioni dei vertici + facce triangolari) scalato **mm → m**.  
5. **Elimina le parti piccole** *(opzionale)* in base alla diagonale del bounding box.  
6. **Decima** *(opzionale)* usando PyMeshLab (preferito) → Open3D → copia di fallback.  
7. **Copia l'OBJ finale** nella destinazione richiesta.

---

## Utilizzo programmatico
```python
from cad_to_obj import convert_cad_to_obj, LIN_DEF_MM, ANG_DEF_RAD, TARGET_FACES, CULL_TINY_MM

convert_cad_to_obj(
    cad_path="part.step",
    out_obj="part.obj",
    lin_def_mm=LIN_DEF_MM,
    ang_def_rad=ANG_DEF_RAD,
    target_faces=TARGET_FACES,
    cull_tiny_mm=CULL_TINY_MM,
)
```

---

## Note
- L'OBJ in uscita è già in **metri**, quindi la scala 1 unit = 1 m di Unity/Unreal è mantenuta.  
- L'orientamento delle facce rimane coerente con le particolarità di OpenCascade.  
- `trimesh` è obbligatorio; la decimazione retrocede gentilmente alla semplice copia quando gli strumenti opzionali mancano.  
- Gli avvisi di `trimesh` (ad esempio quelli legati a SciPy) non bloccano l'esportazione.

---

## Licenza

Rilasciato sotto licenza MIT. Il testo completo è disponibile in `LICENSE`.
