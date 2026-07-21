# Arquitectura — constraints condicionales (ZUPT / NHC)

**Estado:** vigente  
**Fecha:** 2026-07-18  
**Tipo:** recomendación de arquitectura única (no dos notas sueltas)

**Proveniencia:**

- ZUPT por reloj (`forced_time`) — [11-replay-zupt-provenance.md](11-replay-zupt-provenance.md)
- NHC ALWAYS con GNSS — [16-super-tunnel-ieee952-rerun-protocol.md](16-super-tunnel-ieee952-rerun-protocol.md) §5.1–5.2
- Campo Pico: ambos **off** hoy (`pico2_hardware` no arma ZUPT ni NHC)

---

## 1. Principio único

> **ZUPT y NHC, si se activan, deben dispararse por estado del sistema, no por reloj ni por “siempre”.**

El mismo fallo estructural apareció dos veces en el harness:

| Constraint | Política rota | Efecto |
|------------|---------------|--------|
| ZUPT | `forced_time` (t≤30 s ∨ gps_speed bajo) | Comprime `v` / `P_vv` con vehículo en movimiento |
| NHC | ALWAYS (cada tick con GNSS válido) | Arrastra estado ~255 m **antes** del escenario de interés |

Ambos son variantes de: **update agresivo mientras el detector no refleja la física / disponibilidad real**.

---

## 2. Política condicional (diseño objetivo)

Si en el futuro el firmware Pico (u otro target) arma ZUPT o NHC:

| Constraint | Armar solo cuando… | No armar cuando… |
|------------|--------------------|------------------|
| **ZUPT** | Estacionariedad **IMU** (‖a‖≈g ∧ ‖ω‖ bajo), opcionalmente reforzada por speed GNSS si hay fix | Reloj de misión; “primeros N segundos”; speed GNSS solo |
| **NHC** | Ausencia de corrección GNSS reciente (p. ej. no `fix_valid`, o gap > gracia post-seed), y/o régimen de movimiento compatible (vel. const. / no accel agresiva) | Cada tick con GNSS corrigiendo activamente (ALWAYS) |

Gracia post-seed / post-fix: ventana corta tras `ins_ekf_init` o tras accept GNSS en la que NHC permanece off mientras P colapsa — análogo en espíritu a `gap_le_1s` / políticas P_pv, pero aplicado al **disparo** del constraint, no solo a la cruzada.

---

## 3. Campo vs harness (hoy)

| Capa | ZUPT | NHC |
|------|------|-----|
| `pico2_hardware` | Nunca `apply_constraints` / nunca `is_stopping` | `nhc_enabled=false` tras init; nadie lo enciende |
| Replay PC | `--constraint-policy` **obligatorio** (sin default silencioso) | `--nhc-policy` + stride; políticas experimentales en `super_tunnel` |
| `super_tunnel` | Hardcoded `apply_constraints(false, …)` | Políticas OFF / ALWAYS / CONST_VEL / NO_GNSS_FIX |

Urgencia de campo ≈ 0 **hasta** que alguien active estos updates en producción. Esta hoja existe para que esa activación no repita las dos trampas del simulador.

---

## 4. Checklist al activar en hardware

1. Detector basado en **estado** (IMU / GNSS availability), documentado en el PR.  
2. Prueba A/B: misma trayectoria con constraint OFF vs política condicional vs ALWAYS/forced_time (este último solo como control negativo).  
3. Anatomía tick: P_vv / P_pv / drift en los primeros 10 s con GNSS ON.  
4. No reutilizar defaults de replay (`forced_time`) ni banners de benchmark (“Jacobiano corregido”) como garantía de política segura.

---

## 5. Historial

| Versión | Fecha | Notas |
|---------|-------|-------|
| 1.0 | 2026-07-18 | Unifica lecciones ZUPT + NHC en un solo principio de arquitectura |
