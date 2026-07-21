/**
 * @file regression_tests.cpp
 * @brief Punto de entrada standalone para la suite de regresion.
 *
 * Build & run:
 *   cmake --build build --target navicore_regression_test
 *   ./build/navicore_regression_test
 *   ./build/navicore_regression_test --safety-inject
 *
 * También: NaviCore3D_Sim --run-tests
 */

#include "regression_suite.hpp"

#include <cstring>

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--safety-inject") == 0
            || std::strcmp(argv[i], "--quick") == 0) {
            return run_regression_suite_safety_inject();
        }
    }
    return run_regression_suite();
}
