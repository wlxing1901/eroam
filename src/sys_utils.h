#ifndef FX_SYS_UTILS_H
#define FX_SYS_UTILS_H

#include <glog/logging.h>
#include <chrono>

namespace fx {

// Some system-related functions, such as for timing code execution.

/**
 * @brief Measures the execution time of a function.
 * @tparam FuncT
 * @param func The function to be called.
 * @param func_name The name of the function.
 * @param times The number of times to call the function.
 */
template <typename FuncT>
void evaluate_and_call(FuncT&& func, const std::string& func_name = "", int times = 10) {
    double total_time = 0;
    for (int i = 0; i < times; ++i) {
        auto t1 = std::chrono::high_resolution_clock::now();
        func();
        auto t2 = std::chrono::high_resolution_clock::now();
        total_time += std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    }

    LOG(INFO) << "Method " << func_name << " average call time/count: " << total_time / times << "/" << times << " ms.";
}

}  // namespace fx

#endif  // FX_SYS_UTILS_H