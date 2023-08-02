#pragma once

#ifdef ARDUINO
static_assert(false, "fake-serial.h should not be included on Arduino");
#endif

#include <cstdio>
#include <iostream>
#include <string>

class FakeSerial {
 public:
  void print(std::string str) { std::cout << str; }

  void println(std::string str) { std::cout << str << "\n"; }

  void printf(const char* fmt, ...) {
    va_list args;
    vprintf(fmt, args);
  }
};