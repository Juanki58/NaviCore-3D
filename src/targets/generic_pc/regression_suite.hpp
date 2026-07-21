#pragma once

/** Ejecuta la suite completa. Retorna 0 si OK, 1 si hay fallos. */
int run_regression_suite();

/**
 * Suite de inyección safety (NaN IMU, waypoint full, WCET, geometry).
 * Pensada para CI verde (ASan) sin los benchmarks NHC/TC legacy que aún fallan.
 */
int run_regression_suite_safety_inject();
