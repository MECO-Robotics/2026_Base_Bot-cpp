#pragma once
#include <gtest/gtest.h>
#include <string>
#include <string_view>
#include <utility>
namespace frc_test {
template <typename F> void RunScenario(std::string_view name, F&& run) {
  SCOPED_TRACE(std::string(name));
  std::forward<F>(run)();
}
}
