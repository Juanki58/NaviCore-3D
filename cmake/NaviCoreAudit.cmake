# Optional safety / audit build flags for host (PC) targets.
# Include from top-level CMakeLists.txt after project().

option(NAVICORE_ENABLE_SANITIZERS "Address+UB sanitizers on host targets (Clang/GCC)" OFF)
option(NAVICORE_ENABLE_COVERAGE "gcov coverage flags on host targets" OFF)
option(NAVICORE_WARNINGS_AS_ERRORS "Treat warnings as errors on host" OFF)

function(navicore_apply_host_audit_flags target_name)
    if(NOT TARGET ${target_name})
        return()
    endif()

    if(NAVICORE_WARNINGS_AS_ERRORS)
        target_compile_options(${target_name} PRIVATE -Werror)
    endif()

    if(NAVICORE_ENABLE_SANITIZERS)
        if(MSVC)
            message(WARNING "NAVICORE_ENABLE_SANITIZERS: prefer Clang/GCC; MSVC ASan differs")
        else()
            target_compile_options(${target_name} PRIVATE
                -fsanitize=address,undefined
                -fno-omit-frame-pointer
                -g)
            target_link_options(${target_name} PRIVATE
                -fsanitize=address,undefined)
        endif()
    endif()

    if(NAVICORE_ENABLE_COVERAGE)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(${target_name} PRIVATE --coverage -O0 -g)
            target_link_options(${target_name} PRIVATE --coverage)
        else()
            message(WARNING "NAVICORE_ENABLE_COVERAGE requires GCC or Clang")
        endif()
    endif()
endfunction()
