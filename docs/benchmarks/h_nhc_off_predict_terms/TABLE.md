# Predict term budget — Δv_E (#17→#18, NHC off)

| Término | Δv_E (m/s) | En código |
|---------|------------|-----------|
| R·imu (specific force raw→NED) | -6.6532 | sí |
| −R·bias_a | -1.6461 | sí |
| gravity (−g_E) | +0.0000 | sí |
| Coriolis | +0.0000 | no |
| Earth rotation | +0.0000 | no |
| **Σ términos** | **-8.2993** | |
| **Σ a_lin_E·dt (integrador)** | **-8.2993** | |

Dominante: **R·imu** (80.2% del |Σ| in-code)
PASS ≥90% un término: False

Eliminados: Coriolis, Earth rate, gravity_E (=0).