# Reproducir diagnósticos

## Requisitos

- CMake ≥ 3.15, compilador C++17  
- Python 3.10+ con `numpy`, `matplotlib`  
- Datos en `data/real_run/` (Sensor Logger)  
- Binario `build/NaviCore3D_Replay.exe` (Windows) o equivalente

## 1. Compilar replay

```powershell
cmake -S C:\NaviCore-3D -B C:\NaviCore-3D\build
cmake --build C:\NaviCore-3D\build --target NaviCore3D_Replay
```

## 2. Preparar CSV de replay

Si no existe `docs/benchmarks/real_run_replay.csv`:

```powershell
python parse_mobile_log.py --input-dir data/real_run --output docs/benchmarks/real_run_replay.csv
```

## 3. Montaje IMU (opcional regenerar)

```powershell
python audit_imu_chain.py --export-calibration calibration/imu_mount.json
```

## 4. Corrida predict-only base (H9)

```powershell
.\build\NaviCore3D_Replay.exe `
  --input docs/benchmarks/real_run_replay.csv `
  --output docs/benchmarks/h9_predict_only_output.csv `
  --predict-only --predict-only-end-s 60 `
  --h9a-gravity-tilt-init `
  --mount-calibration calibration/imu_mount.json `
  --predict-audit-csv docs/benchmarks/h9_predict_only_audit.csv
```

```powershell
python run_h9_predict_only_isolation.py
```

## 5. Bloque actitud H9a–H9d (orden sugerido)

Cada script invoca el replay con los flags necesarios y escribe informes en `docs/benchmarks/`.

```powershell
python run_h9a_gravity_init.py
python run_h9a_gravity_alignment_audit.py
python run_h9_1_tilt_diagnostic.py
python run_h9b_attitude_propagation_audit.py
python run_h9c_orientation_ref_audit.py
python run_h9d_gravity_subtraction_audit.py
python run_propagation_chain_audit.py
python run_rb_forward_heading_audit.py
```

## 6. Auditorías de convenciones y referencias

Requiere `propagation_chain_audit.csv` (generado por el paso anterior).

```powershell
python run_gravity_triad_audit.py
python audit_attitude_conventions.py
```

Solo sintético:

```powershell
python audit_attitude_conventions.py --synthetic-only
```

Solo cadena empírica:

```powershell
python audit_attitude_conventions.py --empirical-only
```

## 7. Experimentos anteriores (H3–H8)

```powershell
python run_h3_experiment.py
python run_h4_experiment.py
python run_h4_sweep.py
python run_h5_sync_analysis.py
python run_h5_grid_search.py
python run_h6_sync_audit.py
python run_h7_gnss_audit.py
python run_h7_gnss_chain_audit.py
python run_h7b_unified_origin.py
python run_h8_propagation_audit.py
```

## 8. Análisis general real run

```powershell
python analyze_real_run.py `
  --replay docs/benchmarks/real_run_replay.csv `
  --output docs/benchmarks/real_run_output.csv `
  --input-dir data/real_run
```

## Salidas esperadas

| Tipo | Ubicación |
|------|-----------|
| Informes JSON | `docs/benchmarks/*_report.json` |
| Series CSV | `docs/benchmarks/*.csv` |
| Gráficos | `docs/benchmarks/*.png` |
| Documentación | `docs/diagnostics/` |

## Regenerar documentación

La documentación en `docs/diagnostics/` describe los experimentos y resultados de referencia. Tras re-ejecutar scripts, comparar informes JSON actualizados con [04-findings.md](04-findings.md) y actualizar cifras si el dataset cambia.

## Troubleshooting

| Problema | Solución |
|----------|----------|
| Falta `real_run_replay.csv` | Ejecutar `parse_mobile_log.py` |
| Falta `propagation_chain_audit.csv` | Ejecutar `run_propagation_chain_audit.py` |
| `Orientation.csv` no encontrado | Pasar `--input-dir data/real_run` o copiar CSV |
| PowerShell `&&` falla | Usar `; if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }` |
| Unicode en consola | Scripts usan ASCII en stdout |

## Referencia rápida de flags replay

```
--predict-only
--predict-only-end-s <segundos>
--h9a-gravity-tilt-init
--mount-calibration <path.json>
--mount-mode calibration_file
--h9b-attitude-propagation-audit-csv <path>
--h9d-gravity-subtraction-audit-csv <path>
--propagation-chain-audit-csv <path>
--h9a-gravity-alignment-audit-csv <path>
--predict-audit-csv <path>
--disable-gnss / --disable-nhc / --disable-zupt  (según experimento)
```

Ver `replay_main.cpp --help` para lista completa.
