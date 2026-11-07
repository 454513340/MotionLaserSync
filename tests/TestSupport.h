#pragma once
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#define REQUIRE(condition) do { if (!(condition)) throw std::runtime_error( \
    std::string(__FILE__) + ":" + std::to_string(__LINE__) + " " #condition); } while(false)
struct TestCase { const char* name; std::function<void()> run; };
inline int runTests(const std::vector<TestCase>& tests) {
    int failures = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS " << test.name << '\n'; }
        catch (const std::exception& e) { ++failures; std::cerr << "FAIL " << test.name << ": " << e.what() << '\n'; }
    }
    std::cout << tests.size() - failures << "/" << tests.size() << " passed\n";
    return failures == 0 ? 0 : 1;
}
