/**
 * @file regression_tests.cpp
 * @brief Punto de entrada standalone para la suite de regresion.
 *
 * Build & run:
 *   cmake --build build --target navicore_regression_test
 *   ./build/navicore_regression_test
 *
 * Tambien disponible via simulador:
 *   ./build/NaviCore3D_Sim --run-tests
 */

#include "regression_suite.hpp"

int main()
{
    return run_regression_suite();
}
